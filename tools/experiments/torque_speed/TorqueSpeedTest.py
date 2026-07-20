import argparse
import csv
import math
import statistics
import struct
import threading
import time
from pathlib import Path

import serial

try:
    from labjack import ljm
except ImportError:
    ljm = None


DEFAULT_SERIAL_PORT = "COM7"
DEFAULT_BAUD = 921600

# Torque stand / LabJack configuration.
# Matches the standalone torque monitor setup:
#   positive input: AIN2
#   negative input: AIN3
#   differential measurement: AIN2 - AIN3
LABJACK_AIN_DEFAULT = "AIN2"
LABJACK_NEGATIVE_CH_DEFAULT = 3
LABJACK_RANGE_DEFAULT = 10.0
LABJACK_RESOLUTION_INDEX_DEFAULT = 8
LABJACK_TARE_SAMPLES_DEFAULT = 5000

TORQUE_SENSOR_V_PER_NM = 10.0389
PHASE_RESISTANCE_OHM_DEFAULT = 0.132057

UART_MAGIC = b"\xEF\xBE"
UART_FRAME_SIZE = 77
UART_STRUCT = struct.Struct("<H H I 15f B B B B I B")

TELEMETRY_EXPERIMENT_IDLE = 0
TELEMETRY_EXPERIMENT_RUNNING = 1
TELEMETRY_EXPERIMENT_COMPLETE = 2
TELEMETRY_EXPERIMENT_FAULT = 3

TELEMETRY_TRIGGER_BYTE = 0xAA


class SharedData:
    def __init__(self):
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.labjack_ready_event = threading.Event()

        self.zephyr = []
        self.labjack = []
        self.joulescope = []

        self.labjack_tare_voltage = math.nan

        self.final_status = None
        self.final_fault_code = 0


class ZephyrThread(threading.Thread):
    def __init__(self, port, baud, shared, max_duration_s):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.shared = shared
        self.max_duration_s = max_duration_s

        self.ok = 0
        self.bad = 0
        self.timestamp_backwards = 0
        self.raw_bytes = 0

    def run(self):
        self.shared.labjack_ready_event.wait(timeout=15.0)

        ser = serial.Serial(self.port, self.baud, timeout=0.02)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        print("[UART] Sending trigger 0xAA")
        ser.write(bytes([TELEMETRY_TRIGGER_BYTE]))
        ser.flush()

        buf = bytearray()
        last_fw_ts = None
        start = time.perf_counter()

        while not self.shared.stop_event.is_set():
            if time.perf_counter() - start > self.max_duration_s:
                print("[UART] Max duration reached; stopping capture.")
                self.shared.stop_event.set()
                break

            chunk = ser.read(4096)

            if chunk:
                self.raw_bytes += len(chunk)
                buf.extend(chunk)

            while True:
                idx = buf.find(UART_MAGIC)

                if idx < 0:
                    if len(buf) > 1:
                        del buf[:-1]
                    break

                if idx > 0:
                    del buf[:idx]

                if len(buf) < UART_FRAME_SIZE:
                    break

                frame = bytes(buf[:UART_FRAME_SIZE])
                del buf[:UART_FRAME_SIZE]

                if (sum(frame[:76]) & 0xFF) != frame[76]:
                    self.bad += 1
                    continue

                unpacked = UART_STRUCT.unpack(frame)

                (
                    magic,
                    packet_version,
                    fw_ts_us,
                    vbus,
                    ia,
                    ib,
                    ic,
                    omega,
                    speed_cmd,
                    va,
                    vb,
                    vc,
                    temp_inverter,
                    temp_motor,
                    temp_phase_a,
                    temp_phase_b,
                    temp_phase_c,
                    temp_mcu,
                    test_state,
                    comm_mode,
                    experiment_status,
                    fault_code,
                    valid_flags,
                    chk,
                ) = unpacked

                host_time_s = time.perf_counter() - start

                if last_fw_ts is not None and fw_ts_us < last_fw_ts:
                    self.timestamp_backwards += 1

                last_fw_ts = fw_ts_us
                self.ok += 1

                with self.shared.lock:
                    self.shared.zephyr.append([
                        host_time_s,
                        packet_version,
                        fw_ts_us,

                        vbus,
                        ia,
                        ib,
                        ic,

                        omega,
                        speed_cmd,

                        va,
                        vb,
                        vc,

                        temp_inverter,
                        temp_motor,
                        temp_phase_a,
                        temp_phase_b,
                        temp_phase_c,
                        temp_mcu,

                        test_state,
                        comm_mode,
                        experiment_status,
                        fault_code,
                        valid_flags,
                    ])

                    self.shared.final_status = experiment_status
                    self.shared.final_fault_code = fault_code

                if self.ok % 500 == 0:
                    print(
                        f"[UART] ok={self.ok} "
                        f"state={test_state} "
                        f"status={experiment_status} "
                        f"fault={fault_code} "
                        f"omega_rpm={omega * 60.0:.1f} "
                        f"cmd_rpm={speed_cmd * 60.0:.1f}"
                    )

                if experiment_status == TELEMETRY_EXPERIMENT_COMPLETE:
                    print("[UART] Firmware reported COMPLETE.")
                    self.shared.stop_event.set()
                    break

                if experiment_status == TELEMETRY_EXPERIMENT_FAULT:
                    print(f"[UART] Firmware reported FAULT. fault_code={fault_code}")
                    self.shared.stop_event.set()
                    break

        ser.close()


class LabJackThread(threading.Thread):
    def __init__(
        self,
        shared,
        ain_name,
        negative_ch,
        input_range,
        resolution_index,
        tare_samples,
        scan_rate_hz,
        scans_per_read,
    ):
        super().__init__(daemon=True)
        self.shared = shared
        self.ain_name = ain_name
        self.negative_ch = negative_ch
        self.input_range = input_range
        self.resolution_index = resolution_index
        self.tare_samples = tare_samples
        self.scan_rate_hz = scan_rate_hz
        self.scans_per_read = scans_per_read
        self.samples = 0
        self.tare_voltage = math.nan

    def run(self):
        if ljm is None:
            print("[LABJACK] Disabled: labjack-ljm is not installed.")
            self.shared.labjack_ready_event.set()
            return

        handle = None

        try:
            try:
                ljm.closeAll()
            except Exception:
                pass

            handle = ljm.openS("T7", "ANY", "ANY")

            ljm.eWriteName(handle, f"{self.ain_name}_NEGATIVE_CH", self.negative_ch)
            ljm.eWriteName(handle, f"{self.ain_name}_RANGE", self.input_range)
            ljm.eWriteName(handle, f"{self.ain_name}_RESOLUTION_INDEX", self.resolution_index)

            ljm.eWriteName(handle, "STREAM_SETTLING_US", 0)
            ljm.eWriteName(handle, "STREAM_RESOLUTION_INDEX", 0)

            print(
                f"[LABJACK] Configured differential input: "
                f"{self.ain_name} - AIN{self.negative_ch}"
            )
            print(
                f"[LABJACK] Taring torque sensor with {self.tare_samples} samples. "
                f"Keep the stand unloaded/zeroed."
            )

            tare_values = []
            for _ in range(self.tare_samples):
                tare_values.append(ljm.eReadName(handle, self.ain_name))

            self.tare_voltage = statistics.mean(tare_values)

            with self.shared.lock:
                self.shared.labjack_tare_voltage = self.tare_voltage

            print(f"[LABJACK] Tare voltage = {self.tare_voltage:.6f} V")
            self.shared.labjack_ready_event.set()

            addresses, _ = ljm.namesToAddresses(1, [self.ain_name])

            actual_rate = ljm.eStreamStart(
                handle,
                self.scans_per_read,
                1,
                addresses,
                self.scan_rate_hz,
            )

            print(f"[LABJACK] Streaming {self.ain_name} at {actual_rate:.1f} Hz")

            start = time.perf_counter()
            sample_dt = 1.0 / actual_rate

            while not self.shared.stop_event.is_set():
                data, _dev_backlog, _ljm_backlog = ljm.eStreamRead(handle)
                read_time_s = time.perf_counter() - start

                n = len(data)

                if n <= 0:
                    continue

                first_time_s = read_time_s - ((n - 1) * sample_dt)

                rows = []

                for i, voltage in enumerate(data):
                    host_time_s = first_time_s + i * sample_dt
                    torque_voltage = float(voltage) - self.tare_voltage
                    torque_nm = torque_voltage / TORQUE_SENSOR_V_PER_NM

                    rows.append([
                        host_time_s,
                        float(voltage),
                        float(torque_voltage),
                        float(torque_nm),
                    ])

                with self.shared.lock:
                    self.shared.labjack.extend(rows)

                self.samples += n

        except Exception as e:
            print("[LABJACK] Capture failed.")
            print(e)
            self.shared.labjack_ready_event.set()

        finally:
            if handle is not None:
                try:
                    ljm.eStreamStop(handle)
                except Exception:
                    pass

                try:
                    ljm.close(handle)
                except Exception:
                    pass


class JoulescopeThread(threading.Thread):
    def __init__(self, shared, sample_hz):
        super().__init__(daemon=True)
        self.shared = shared
        self.sample_hz = sample_hz
        self.samples = 0

    def run(self):
        try:
            import numpy as np
            import joulescope
        except ImportError:
            print("[JOULESCOPE] Disabled: install joulescope package.")
            return

        device = None

        try:
            time.sleep(2.0)
            device = joulescope.scan_require_one()

            block_s = 0.1
            start = time.perf_counter()

            print("[JOULESCOPE] Capture started.")

            with device:
                while not self.shared.stop_event.is_set():
                    block_start = time.perf_counter()

                    data = device.read(
                        duration=block_s,
                        out_format="samples_get",
                        fields=["current", "voltage"],
                    )

                    current = data["signals"]["current"]["value"]
                    voltage = data["signals"]["voltage"]["value"]

                    n = min(len(current), len(voltage))

                    if n <= 0:
                        continue

                    current = np.asarray(current[:n], dtype=float)
                    voltage = np.asarray(voltage[:n], dtype=float)
                    power = voltage * current

                    raw_rate = n / block_s
                    step = max(1, int(raw_rate / self.sample_hz))

                    rows = []

                    for i in range(0, n, step):
                        host_time_s = (block_start - start) + (i / raw_rate)

                        rows.append([
                            host_time_s,
                            float(voltage[i]),
                            float(current[i]),
                            float(power[i]),
                        ])

                    with self.shared.lock:
                        self.shared.joulescope.extend(rows)

                    self.samples += len(rows)

        except Exception as e:
            print("[JOULESCOPE] Capture failed.")
            print(e)

        finally:
            try:
                if device is not None:
                    device.close()
            except Exception:
                pass



def interp_at(t, source_rows, value_index):
    if not source_rows:
        return math.nan

    if t <= source_rows[0][0]:
        return source_rows[0][value_index]

    if t >= source_rows[-1][0]:
        return source_rows[-1][value_index]

    lo = 0
    hi = len(source_rows) - 1

    while hi - lo > 1:
        mid = (lo + hi) // 2

        if source_rows[mid][0] <= t:
            lo = mid
        else:
            hi = mid

    t0 = source_rows[lo][0]
    t1 = source_rows[hi][0]
    y0 = source_rows[lo][value_index]
    y1 = source_rows[hi][value_index]

    if t1 == t0:
        return y0

    frac = (t - t0) / (t1 - t0)
    return y0 + frac * (y1 - y0)


def write_csv(path, shared, output_rate_hz, phase_resistance_ohm):
    with shared.lock:
        zephyr = list(shared.zephyr)
        labjack = list(shared.labjack)
        joulescope = list(shared.joulescope)
        labjack_tare_voltage = shared.labjack_tare_voltage

    zephyr.sort(key=lambda r: r[0])
    labjack.sort(key=lambda r: r[0])
    joulescope.sort(key=lambda r: r[0])

    if not zephyr:
        raise RuntimeError("No Zephyr telemetry captured.")

    duration_s = zephyr[-1][0]
    n = int(duration_s * output_rate_hz) + 1
    dt = 1.0 / output_rate_hz

    with open(path, "w", newline="") as f:
        writer = csv.writer(f)

        writer.writerow([
            "host_time_s",
            "packet_version",
            "zephyr_fw_timestamp_us",

            "vbus_V",
            "ia_A",
            "ib_A",
            "ic_A",

            "omega_rev_s",
            "omega_rad_s",
            "omega_rpm",

            "speed_command_rev_s",
            "speed_command_rpm",

            "va_V",
            "vb_V",
            "vc_V",

            "temp_inverter_C",
            "temp_motor_C",
            "temp_phase_a_C",
            "temp_phase_b_C",
            "temp_phase_c_C",
            "temp_mcu_C",

            "test_state",
            "commutation_mode",
            "experiment_status",
            "fault_code",
            "valid_flags",

            "torque_sensor_V",
            "torque_sensor_tare_V",
            "torque_sensor_delta_V",
            "torque_Nm",

            "js_voltage_V",
            "js_current_A",
            "js_power_W",

            "p_dc_in_W",
            "p_mech_W",
            "p_cu_W",

            "torque_speed_abs_rpm",
            "torque_speed_signed_rpm",
            "torque_speed_abs_rad_s",
        ])

        for i in range(n):
            t = i * dt

            packet_version = interp_at(t, zephyr, 1)
            fw_ts = interp_at(t, zephyr, 2)

            vbus = interp_at(t, zephyr, 3)
            ia = interp_at(t, zephyr, 4)
            ib = interp_at(t, zephyr, 5)
            ic = interp_at(t, zephyr, 6)

            omega_rev_s = interp_at(t, zephyr, 7)
            speed_cmd_rev_s = interp_at(t, zephyr, 8)

            va = interp_at(t, zephyr, 9)
            vb = interp_at(t, zephyr, 10)
            vc = interp_at(t, zephyr, 11)

            temp_inverter = interp_at(t, zephyr, 12)
            temp_motor = interp_at(t, zephyr, 13)
            temp_phase_a = interp_at(t, zephyr, 14)
            temp_phase_b = interp_at(t, zephyr, 15)
            temp_phase_c = interp_at(t, zephyr, 16)
            temp_mcu = interp_at(t, zephyr, 17)

            test_state = interp_at(t, zephyr, 18)
            comm_mode = interp_at(t, zephyr, 19)
            experiment_status = interp_at(t, zephyr, 20)
            fault_code = interp_at(t, zephyr, 21)
            valid_flags = interp_at(t, zephyr, 22)

            torque_v = interp_at(t, labjack, 1)
            torque_delta_v = interp_at(t, labjack, 2)
            torque_nm = interp_at(t, labjack, 3)

            js_v = interp_at(t, joulescope, 1)
            js_i = interp_at(t, joulescope, 2)
            js_p = interp_at(t, joulescope, 3)

            omega_rad_s = omega_rev_s * 2.0 * math.pi if math.isfinite(omega_rev_s) else math.nan
            omega_rpm = omega_rev_s * 60.0 if math.isfinite(omega_rev_s) else math.nan
            speed_cmd_rpm = speed_cmd_rev_s * 60.0 if math.isfinite(speed_cmd_rev_s) else math.nan

            if math.isfinite(vbus) and math.isfinite(js_i):
                p_dc_in = vbus * js_i
            else:
                p_dc_in = math.nan

            if math.isfinite(torque_nm) and math.isfinite(omega_rad_s):
                p_mech = torque_nm * omega_rad_s
            else:
                p_mech = math.nan

            if all(math.isfinite(x) for x in [ia, ib, ic]):
                p_cu = phase_resistance_ohm * (ia * ia + ib * ib + ic * ic)
            else:
                p_cu = math.nan

            torque_speed_abs_rpm = abs(omega_rpm) if math.isfinite(omega_rpm) else math.nan
            torque_speed_signed_rpm = omega_rpm
            torque_speed_abs_rad_s = abs(omega_rad_s) if math.isfinite(omega_rad_s) else math.nan

            writer.writerow([
                t,
                packet_version,
                fw_ts,

                vbus,
                ia,
                ib,
                ic,

                omega_rev_s,
                omega_rad_s,
                omega_rpm,

                speed_cmd_rev_s,
                speed_cmd_rpm,

                va,
                vb,
                vc,

                temp_inverter,
                temp_motor,
                temp_phase_a,
                temp_phase_b,
                temp_phase_c,
                temp_mcu,

                round(test_state) if math.isfinite(test_state) else "",
                round(comm_mode) if math.isfinite(comm_mode) else "",
                round(experiment_status) if math.isfinite(experiment_status) else "",
                round(fault_code) if math.isfinite(fault_code) else "",
                round(valid_flags) if math.isfinite(valid_flags) else "",

                torque_v,
                labjack_tare_voltage,
                torque_delta_v,
                torque_nm,

                js_v,
                js_i,
                js_p,

                p_dc_in,
                p_mech,
                p_cu,

                torque_speed_abs_rpm,
                torque_speed_signed_rpm,
                torque_speed_abs_rad_s,
            ])


def main():
    parser = argparse.ArgumentParser(description="TorqueSpeedTest DAQ capture")

    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--out", default="torque_speed_raw.csv")

    parser.add_argument("--output-rate-hz", type=float, default=200.0)
    parser.add_argument("--max-duration-s", type=float, default=180.0)

    parser.add_argument("--labjack-ain", default=LABJACK_AIN_DEFAULT)
    parser.add_argument("--labjack-negative-ch", type=int, default=LABJACK_NEGATIVE_CH_DEFAULT)
    parser.add_argument("--labjack-range", type=float, default=LABJACK_RANGE_DEFAULT)
    parser.add_argument("--labjack-resolution-index", type=int, default=LABJACK_RESOLUTION_INDEX_DEFAULT)
    parser.add_argument("--labjack-tare-samples", type=int, default=LABJACK_TARE_SAMPLES_DEFAULT)

    parser.add_argument("--labjack-rate-hz", type=float, default=1000.0)
    parser.add_argument("--labjack-scans-per-read", type=int, default=100)

    parser.add_argument("--joulescope-rate-hz", type=float, default=1000.0)

    parser.add_argument("--phase-resistance-ohm", type=float, default=PHASE_RESISTANCE_OHM_DEFAULT)

    args = parser.parse_args()

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    shared = SharedData()

    zephyr = ZephyrThread(
        port=DEFAULT_SERIAL_PORT,
        baud=args.baud,
        shared=shared,
        max_duration_s=args.max_duration_s,
    )

    labjack = LabJackThread(
        shared=shared,
        ain_name=args.labjack_ain,
        negative_ch=args.labjack_negative_ch,
        input_range=args.labjack_range,
        resolution_index=args.labjack_resolution_index,
        tare_samples=args.labjack_tare_samples,
        scan_rate_hz=args.labjack_rate_hz,
        scans_per_read=args.labjack_scans_per_read,
    )

    joulescope = JoulescopeThread(
        shared=shared,
        sample_hz=args.joulescope_rate_hz
    )

    print("[DAQ] Starting acquisition threads.")
    labjack.start()
    joulescope.start()
    zephyr.start()

    zephyr.join()

    shared.stop_event.set()

    labjack.join(timeout=2.0)
    joulescope.join(timeout=2.0)

    print(f"[UART] ok={zephyr.ok} bad={zephyr.bad} raw_bytes={zephyr.raw_bytes} "
          f"timestamp_backwards={zephyr.timestamp_backwards}")

    print("[DAQ] Writing CSV.")
    write_csv(
        path=out_path,
        shared=shared,
        output_rate_hz=args.output_rate_hz,
        phase_resistance_ohm=args.phase_resistance_ohm,
    )

    print("[DAQ] Done.")
    print(f"[DAQ] Output: {out_path}")
    print(f"[LABJACK] samples={labjack.samples}")
    print(f"[LABJACK] tare_voltage={shared.labjack_tare_voltage}")
    print(f"[JOULESCOPE] samples={joulescope.samples}")
    print(f"[DAQ] final_status={shared.final_status} final_fault_code={shared.final_fault_code}")


if __name__ == "__main__":
    main()
