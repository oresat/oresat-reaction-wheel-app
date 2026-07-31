"""
PowerRippleResults.py

Processing and plotting for the OreSat reaction-wheel PowerRippleTest.

This revision intentionally limits the analysis to quantities supported by the
available instrumentation during steady-speed plateaus. The torque stand
measures reaction torque, not loaded shaft-output torque; therefore this script
does not report mechanical output power, motor efficiency, or residual loss.

Primary figures:
    power_input_vs_time.png/.pdf
    power_input_vs_rpm.png/.pdf
    total_energy_consumption.png/.pdf
    phase_current_rms_vs_rpm.png/.pdf
    copper_loss_vs_rpm.png/.pdf
    velocity_stability_vs_rpm.png/.pdf
    torque_ripple_rms_vs_rpm.png/.pdf
    torque_ripple_robust_ptp_vs_rpm.png/.pdf

Important output CSVs:
    power_ripple_timeseries_processed.csv
    power_ripple_timeseries_aggregate.csv
    power_ripple_plateau_runs.csv
    power_ripple_plateau_aggregate_all.csv
    power_ripple_run_summary.csv
    power_ripple_experiment_summary.csv
    power_ripple_processing_settings.csv

Torque-variation methodology:
    1. Reject isolated slew-rate and Hampel outliers.
    2. Use the Hampel-cleaned torque signal before the rolling display mean.
    3. Remove the constant offset and slow linear drift within each plateau.
    4. Report RMS variation and a robust 95th-to-5th percentile span.

The LabJack tare is trusted by default. Low-speed re-zeroing is disabled unless
--enable-zero-torque-reference is explicitly supplied.
"""

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# =============================================================================
# DEFAULT SETTINGS
# =============================================================================

DEFAULT_OUTDIR = "Figures"
DEFAULT_BIN_WIDTH_RPM = 500.0
DEFAULT_MIN_SAMPLES_PER_BIN = 50
DEFAULT_MAX_TORQUE_SLEW_MNM_S = 10.0
DEFAULT_HAMPEL_WINDOW_S = 0.25
DEFAULT_HAMPEL_N_SIGMA = 6.0
DEFAULT_MEAN_WINDOW_S = 0.5
DEFAULT_FINAL_BIN_SMOOTH_WINDOW = 1
DEFAULT_ZERO_REFERENCE_MAX_RPM = 0.0
DEFAULT_PLOT_COLUMN = "torque_median_filtered_Nm"

DEFAULT_PHASE_RESISTANCE_OHM = 196.0e-3 / 2.0

STRATEGY_INPUTS = {
    "Trapezoidal": "trap",
    "Sinusoidal": "sine",
    "FOC": "foc",
}

STRATEGY_ORDER = ["Trapezoidal", "Sinusoidal", "FOC"]

STRATEGY_COLORS = {
    "Trapezoidal": "#1f77b4",   # blue
    "Sinusoidal":  "#ff7f0e",   # orange
    "FOC":         "#2ca02c",   # green
}


# =============================================================================
# INPUT LOADING
# =============================================================================

def read_csv_checked(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)

    required = ["host_time_s", "omega_rpm", "speed_command_rpm"]
    for column in required:
        if column not in df.columns:
            raise ValueError(f"{path} is missing required column: {column}")

    df = df.copy()

    numeric_columns = [
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
        "eta_global",
    ]

    for column in numeric_columns:
        if column in df.columns:
            df[column] = pd.to_numeric(df[column], errors="coerce")

    if "omega_rad_s" not in df.columns:
        df["omega_rad_s"] = pd.to_numeric(df["omega_rpm"], errors="coerce") * 2.0 * np.pi / 60.0

    if "torque_Nm" not in df.columns:
        df["torque_Nm"] = np.nan

    df = df[np.isfinite(df["host_time_s"])]
    return df.sort_values("host_time_s").reset_index(drop=True)


def collect_inputs(args) -> list[dict]:
    jobs = []

    for strategy, attr in STRATEGY_INPUTS.items():
        paths = getattr(args, attr)
        if not paths:
            continue

        for run_index, path_str in enumerate(paths, start=1):
            jobs.append({
                "strategy": strategy,
                "run_index": run_index,
                "path": Path(path_str),
            })

    if not jobs:
        raise ValueError("No input CSV files provided. Pass at least one of --trap, --sine, or --foc.")

    return jobs


def detect_joulescope_status(df: pd.DataFrame) -> bool:
    """Return True only when usable nonzero Joulescope power data exists."""
    if "js_power_W" in df.columns:
        power = pd.to_numeric(df["js_power_W"], errors="coerce").to_numpy(dtype=float)
        power = power[np.isfinite(power)]

        if len(power) > 0 and np.nanmax(np.abs(power)) > 1.0e-6:
            return True

    if "js_voltage_V" in df.columns and "js_current_A" in df.columns:
        voltage = pd.to_numeric(df["js_voltage_V"], errors="coerce").to_numpy(dtype=float)
        current = pd.to_numeric(df["js_current_A"], errors="coerce").to_numpy(dtype=float)

        mask = np.isfinite(voltage) & np.isfinite(current)
        if np.any(mask):
            calculated_power = voltage[mask] * current[mask]
            if np.nanmax(np.abs(calculated_power)) > 1.0e-6:
                return True

    return False


# =============================================================================
# NUMERICAL HELPERS
# =============================================================================

def estimate_sample_period_s(df: pd.DataFrame) -> float:
    t = pd.to_numeric(df["host_time_s"], errors="coerce").to_numpy(dtype=float)
    t = t[np.isfinite(t)]

    if len(t) < 3:
        return np.nan

    dt = np.diff(t)
    dt = dt[np.isfinite(dt)]
    dt = dt[dt > 0.0]

    if len(dt) == 0:
        return np.nan

    return float(np.median(dt))


def make_odd(n: int) -> int:
    if n <= 1:
        return 1
    return n if n % 2 else n + 1


def window_seconds_to_samples(window_s: float, dt_s: float) -> int:
    if not np.isfinite(window_s) or window_s <= 0.0:
        return 1
    if not np.isfinite(dt_s) or dt_s <= 0.0:
        return 1
    return max(1, int(round(window_s / dt_s)))


def iqr(values) -> float:
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]

    if len(values) == 0:
        return np.nan

    q75, q25 = np.nanpercentile(values, [75.0, 25.0])
    return float(q75 - q25)


def finite_any(values) -> bool:
    arr = np.asarray(values, dtype=float)
    return bool(np.isfinite(arr).any())


def finite_mean(values) -> float:
    arr = np.asarray(values, dtype=float)
    arr = arr[np.isfinite(arr)]
    if len(arr) == 0:
        return np.nan
    return float(np.nanmean(arr))


def finite_std(values) -> float:
    arr = np.asarray(values, dtype=float)
    arr = arr[np.isfinite(arr)]
    if len(arr) <= 1:
        return 0.0 if len(arr) == 1 else np.nan
    return float(np.nanstd(arr, ddof=1))


def has_usable_power_data(df: pd.DataFrame) -> bool:
    if "p_dc_in_calc_W" not in df.columns:
        return False
    return finite_any(pd.to_numeric(df["p_dc_in_calc_W"], errors="coerce"))


def integrate_trapezoid(t, y) -> float:
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)

    mask = np.isfinite(t) & np.isfinite(y)
    if np.sum(mask) < 2:
        return np.nan

    if hasattr(np, "trapezoid"):
        return float(np.trapezoid(y[mask], t[mask]))

    return float(np.trapz(y[mask], t[mask]))


def rolling_centered(values: pd.Series, window: int, use_median: bool = False) -> pd.Series:
    y = pd.to_numeric(values, errors="coerce").astype(float)
    window = make_odd(int(window))

    if window <= 1:
        return y

    roller = y.rolling(window=window, center=True, min_periods=max(1, window // 3))
    out = roller.median() if use_median else roller.mean()
    return out.interpolate(limit_direction="both").bfill().ffill()


def rolling_mean_by_time(values: pd.Series, dt_s: float, window_s: float) -> pd.Series:
    n = make_odd(window_seconds_to_samples(window_s, dt_s))
    return rolling_centered(values, n, use_median=False)


# =============================================================================
# TORQUE FILTERING
# =============================================================================

def reject_slew_spikes(values: pd.Series, time_s: pd.Series, max_slew_mnm_s: float, passes: int = 2):
    y_original = pd.to_numeric(values, errors="coerce").astype(float)
    t = pd.to_numeric(time_s, errors="coerce").astype(float)

    if not np.isfinite(max_slew_mnm_s) or max_slew_mnm_s <= 0.0:
        return y_original, pd.Series(False, index=y_original.index)

    max_slew_nm_s = max_slew_mnm_s / 1000.0

    y = y_original.copy()
    flags_total = pd.Series(False, index=y.index)

    for _ in range(max(1, passes)):
        y_work = y.copy()
        flags = pd.Series(False, index=y.index)

        last_t = None
        last_y = None

        for idx in y.index:
            yi = y.loc[idx]
            ti = t.loc[idx]

            if not np.isfinite(yi) or not np.isfinite(ti):
                continue

            if last_t is None:
                last_t = ti
                last_y = yi
                continue

            dt = ti - last_t
            if not np.isfinite(dt) or dt <= 0.0:
                last_t = ti
                last_y = yi
                continue

            if abs(yi - last_y) > max_slew_nm_s * dt:
                flags.loc[idx] = True
                y_work.loc[idx] = np.nan
                continue

            last_t = ti
            last_y = yi

        flags_total |= flags
        y = y_work.interpolate(limit_direction="both").bfill().ffill()

    return y, flags_total


def hampel_filter(values: pd.Series, dt_s: float, window_s: float, n_sigma: float):
    y = pd.to_numeric(values, errors="coerce").astype(float)
    n = make_odd(window_seconds_to_samples(window_s, dt_s))

    if n <= 1:
        return y, pd.Series(False, index=y.index)

    local_median = y.rolling(window=n, center=True, min_periods=max(1, n // 3)).median()
    abs_dev = (y - local_median).abs()
    local_mad = abs_dev.rolling(window=n, center=True, min_periods=max(1, n // 3)).median()

    robust_sigma = 1.4826 * local_mad
    threshold = n_sigma * robust_sigma

    flags = (
        (abs_dev > threshold) &
        np.isfinite(abs_dev) &
        np.isfinite(threshold) &
        (threshold > 0.0)
    )

    cleaned = y.copy()
    cleaned.loc[flags] = local_median.loc[flags]
    cleaned = cleaned.interpolate(limit_direction="both").bfill().ffill()

    return cleaned, flags


def process_torque(df: pd.DataFrame, args) -> pd.DataFrame:
    out = df.copy()
    dt = estimate_sample_period_s(out)

    raw = pd.to_numeric(out["torque_Nm"], errors="coerce").astype(float)
    out["torque_raw_Nm"] = raw

    if args.no_torque_filter:
        out["torque_slew_clean_Nm"] = raw
        out["torque_hampel_clean_Nm"] = raw
        out["torque_filtered_Nm"] = raw
        out["torque_rejected_flag"] = 0
        return out

    slew, slew_flags = reject_slew_spikes(
        values=raw,
        time_s=out["host_time_s"],
        max_slew_mnm_s=args.max_torque_slew_mnm_s,
        passes=2,
    )

    hampel, hampel_flags = hampel_filter(
        values=slew,
        dt_s=dt,
        window_s=args.hampel_window_s,
        n_sigma=args.hampel_n_sigma,
    )

    filtered = rolling_mean_by_time(hampel, dt, args.mean_window_s)

    out["torque_slew_clean_Nm"] = slew
    out["torque_hampel_clean_Nm"] = hampel
    out["torque_filtered_Nm"] = filtered
    out["torque_rejected_flag"] = (slew_flags | hampel_flags).astype(int)

    return out


def apply_low_speed_torque_zero(df: pd.DataFrame, args) -> tuple[pd.DataFrame, float]:
    out = df.copy()

    if args.no_zero_torque_reference:
        out["torque_zeroed_Nm"] = out["torque_filtered_Nm"]
        return out, 0.0

    speed = pd.to_numeric(out["omega_rpm"], errors="coerce").abs()
    torque = pd.to_numeric(out["torque_filtered_Nm"], errors="coerce")

    mask = (
        np.isfinite(speed) &
        np.isfinite(torque) &
        (speed <= args.zero_reference_max_rpm)
    )

    if mask.any():
        baseline = float(np.nanmedian(torque[mask]))
    else:
        baseline = 0.0

    out["torque_zeroed_Nm"] = out["torque_filtered_Nm"] - baseline
    return out, baseline


# =============================================================================
# DERIVED QUANTITIES
# =============================================================================

def compute_derived_quantities(df: pd.DataFrame, args) -> pd.DataFrame:
    """Compute only directly supported electrical quantities."""
    out = df.copy()

    nan_series = pd.Series(np.nan, index=out.index, dtype=float)

    vbus = pd.to_numeric(out.get("vbus_V", nan_series), errors="coerce")
    js_voltage = pd.to_numeric(out.get("js_voltage_V", nan_series), errors="coerce")
    js_current = pd.to_numeric(out.get("js_current_A", nan_series), errors="coerce")
    js_power = pd.to_numeric(out.get("js_power_W", nan_series), errors="coerce")

    js_power_values = js_power.to_numpy(dtype=float)
    usable_direct_power = (
        np.isfinite(js_power_values).any()
        and np.nanmax(np.abs(js_power_values)) > 1.0e-6
    )

    if usable_direct_power:
        out["p_dc_in_calc_W"] = js_power

    elif (
        np.isfinite(js_voltage.to_numpy(dtype=float)).any()
        and np.isfinite(js_current.to_numpy(dtype=float)).any()
    ):
        out["p_dc_in_calc_W"] = js_voltage * js_current

    elif "p_dc_in_W" in out.columns:
        fallback = pd.to_numeric(out["p_dc_in_W"], errors="coerce")
        if np.isfinite(fallback.to_numpy(dtype=float)).any():
            out["p_dc_in_calc_W"] = fallback
        else:
            out["p_dc_in_calc_W"] = np.nan

    else:
        out["p_dc_in_calc_W"] = np.nan

    ia = pd.to_numeric(out.get("ia_A", nan_series), errors="coerce")
    ib = pd.to_numeric(out.get("ib_A", nan_series), errors="coerce")
    ic = pd.to_numeric(out.get("ic_A", nan_series), errors="coerce")

    phase_square_mean = (ia * ia + ib * ib + ic * ic) / 3.0
    out["phase_current_rms_instant_A"] = np.sqrt(
        np.maximum(phase_square_mean, 0.0)
    )

    # Three-phase copper loss for three inline phase-current measurements.
    out["p_cu_calc_W"] = args.phase_resistance_ohm * (
        ia * ia + ib * ib + ic * ic
    )

    return out

# =============================================================================
# PLATEAU EXTRACTION AND STATS
# =============================================================================

def identify_plateaus(df: pd.DataFrame, args) -> pd.DataFrame:
    out = df.copy()

    print(f"[PLATEAU] Input rows: {len(out)}")

    speed_cmd = pd.to_numeric(out["speed_command_rpm"], errors="coerce")
    rounded = (
        np.round(speed_cmd / args.bin_width_rpm)
        * args.bin_width_rpm
    ).astype(float)

    out["plateau_rpm"] = rounded

    out = out[np.isfinite(out["plateau_rpm"])]
    out = out[out["plateau_rpm"] > 0.0]

    print(f"[PLATEAU] Positive-command rows: {len(out)}")

    if "test_state" in out.columns:
        state = pd.to_numeric(out["test_state"], errors="coerce")

        state_counts = (
            state.dropna()
            .round()
            .astype(int)
            .value_counts()
            .sort_index()
        )

        if not state_counts.empty:
            print("[PLATEAU] Test-state counts:")
            print(state_counts.to_string())

        plateau_state_mask = np.isfinite(state) & (np.round(state) == 2)

        if plateau_state_mask.any():
            out = out[plateau_state_mask]
            print(f"[PLATEAU] State-2 rows: {len(out)}")
        else:
            print(
                "[WARN] No test_state == 2 samples found; "
                "using speed-based plateau detection only."
            )

    speed = pd.to_numeric(out["omega_rpm"], errors="coerce")
    speed_error = np.abs(speed - out["plateau_rpm"])

    # Allow modest steady-state tracking error without discarding the entire
    # plateau, while still rejecting ramps and clearly unsettled samples.
    tolerance = np.maximum(150.0, 0.05 * out["plateau_rpm"])

    out = out[
        np.isfinite(speed)
        & (speed_error <= tolerance)
    ]

    print(f"[PLATEAU] Rows after speed tolerance: {len(out)}")

    if not out.empty:
        counts = out.groupby("plateau_rpm").size().sort_index()
        print("[PLATEAU] Samples per plateau:")
        print(counts.to_string())

    return out


def stats_for_values(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]

    if len(values) == 0:
        return {
            "mean": np.nan,
            "median": np.nan,
            "std": np.nan,
            "min": np.nan,
            "max": np.nan,
            "ptp": np.nan,
            "iqr": np.nan,
            "samples": 0,
        }

    return {
        "mean": float(np.nanmean(values)),
        "median": float(np.nanmedian(values)),
        "std": float(np.nanstd(values, ddof=1)) if len(values) > 1 else 0.0,
        "min": float(np.nanmin(values)),
        "max": float(np.nanmax(values)),
        "ptp": float(np.nanmax(values) - np.nanmin(values)),
        "iqr": iqr(values),
        "samples": int(len(values)),
    }


def plateau_stats_for_run(df: pd.DataFrame, job: dict, args) -> pd.DataFrame:
    plateau_df = identify_plateaus(df, args)
    rows = []

    for plateau_rpm, g in plateau_df.groupby("plateau_rpm"):
        if len(g) < args.min_samples_per_bin:
            continue

        omega = pd.to_numeric(g["omega_rpm"], errors="coerce").to_numpy(dtype=float)
        torque = pd.to_numeric(g["torque_zeroed_Nm"], errors="coerce").to_numpy(dtype=float)
        pdc = pd.to_numeric(g["p_dc_in_calc_W"], errors="coerce").to_numpy(dtype=float)
        pcu = pd.to_numeric(g["p_cu_calc_W"], errors="coerce").to_numpy(dtype=float)

        ia = pd.to_numeric(g.get("ia_A", np.nan), errors="coerce").to_numpy(dtype=float)
        ib = pd.to_numeric(g.get("ib_A", np.nan), errors="coerce").to_numpy(dtype=float)
        ic = pd.to_numeric(g.get("ic_A", np.nan), errors="coerce").to_numpy(dtype=float)

        current_valid = np.isfinite(ia) & np.isfinite(ib) & np.isfinite(ic)
        if np.any(current_valid):
            phase_current_rms_A = float(np.sqrt(np.nanmean(
                (ia[current_valid] ** 2 + ib[current_valid] ** 2 + ic[current_valid] ** 2) / 3.0
            )))
        else:
            phase_current_rms_A = np.nan

        # Ripple is calculated from the outlier-cleaned signal before the
        # rolling mean, so the displayed ripple is not artificially suppressed.
        ripple_column = (
            "torque_hampel_clean_Nm"
            if "torque_hampel_clean_Nm" in g.columns
            else "torque_filtered_Nm"
        )
        ripple_source = pd.to_numeric(g[ripple_column], errors="coerce").to_numpy(dtype=float)
        ripple_source = ripple_source[np.isfinite(ripple_source)]

        ripple_frame = g[["host_time_s", ripple_column]].copy()
        ripple_frame["host_time_s"] = pd.to_numeric(
            ripple_frame["host_time_s"], errors="coerce"
        )
        ripple_frame[ripple_column] = pd.to_numeric(
            ripple_frame[ripple_column], errors="coerce"
        )
        ripple_frame = ripple_frame.dropna()

        if len(ripple_frame) >= 3:
            t_rel = (
                ripple_frame["host_time_s"].to_numpy(dtype=float)
                - float(ripple_frame["host_time_s"].iloc[0])
            )
            torque_values = ripple_frame[ripple_column].to_numpy(dtype=float)

            # Remove constant offset and slow linear drift without suppressing
            # periodic reaction-torque content within the plateau.
            slope, intercept = np.polyfit(t_rel, torque_values, deg=1)
            torque_trend = slope * t_rel + intercept
            ripple_residual = torque_values - torque_trend

            torque_ripple_rms_Nm = float(
                np.sqrt(np.mean(np.square(ripple_residual)))
            )
            torque_ripple_robust_ptp_Nm = float(
                np.percentile(ripple_residual, 95.0)
                - np.percentile(ripple_residual, 5.0)
            )
            torque_ripple_std_Nm = float(
                np.std(ripple_residual, ddof=1)
            )
        else:
            torque_ripple_rms_Nm = np.nan
            torque_ripple_robust_ptp_Nm = np.nan
            torque_ripple_std_Nm = np.nan

        omega_s = stats_for_values(omega)
        torque_s = stats_for_values(torque)
        pdc_s = stats_for_values(pdc)
        pcu_s = stats_for_values(pcu)

        rows.append({
            "strategy": job["strategy"],
            "run_index": job["run_index"],
            "run_id": f"{job['strategy']}_run{job['run_index']}",
            "source_file": str(job["path"]),
            "plateau_rpm": float(plateau_rpm),
            "samples": int(len(g)),

            "omega_mean_rpm": omega_s["mean"],
            "omega_std_rpm": omega_s["std"],
            "omega_ptp_rpm": omega_s["ptp"],

            "torque_mean_Nm": torque_s["mean"],
            "torque_median_Nm": torque_s["median"],
            "torque_std_Nm": torque_s["std"],
            "torque_ptp_Nm": torque_s["ptp"],
            "torque_ripple_rms_Nm": torque_ripple_rms_Nm,
            "torque_ripple_std_Nm": torque_ripple_std_Nm,
            "torque_ripple_robust_ptp_Nm": torque_ripple_robust_ptp_Nm,

            "phase_current_rms_A": phase_current_rms_A,
            "p_dc_mean_W": pdc_s["mean"],
            "p_dc_median_W": pdc_s["median"],
            "p_cu_mean_W": pcu_s["mean"],
            "p_cu_median_W": pcu_s["median"],
        })

    return pd.DataFrame(rows)


def aggregate_plateaus(plateau_runs: pd.DataFrame, args) -> pd.DataFrame:
    if plateau_runs.empty:
        return pd.DataFrame()

    metric_cols = [
        "omega_mean_rpm",
        "omega_std_rpm",
        "omega_ptp_rpm",
        "torque_mean_Nm",
        "torque_median_Nm",
        "torque_std_Nm",
        "torque_ptp_Nm",
        "torque_ripple_rms_Nm",
        "torque_ripple_std_Nm",
        "torque_ripple_robust_ptp_Nm",
        "phase_current_rms_A",
        "p_dc_mean_W",
        "p_dc_median_W",
        "p_cu_mean_W",
        "p_cu_median_W",
    ]

    rows = []

    for (strategy, rpm), g in plateau_runs.groupby(["strategy", "plateau_rpm"]):
        row = {
            "strategy": strategy,
            "plateau_rpm": float(rpm),
            "run_count": int(g["run_id"].nunique()),
            "total_sample_count": int(np.nansum(g["samples"])),
        }

        for col in metric_cols:
            vals = pd.to_numeric(g[col], errors="coerce").to_numpy(dtype=float)
            vals = vals[np.isfinite(vals)]

            row[f"{col}_mean"] = float(np.nanmean(vals)) if len(vals) else np.nan
            row[f"{col}_median"] = float(np.nanmedian(vals)) if len(vals) else np.nan
            row[f"{col}_std"] = (
                float(np.nanstd(vals, ddof=1))
                if len(vals) > 1
                else 0.0 if len(vals) == 1 else np.nan
            )
            row[f"{col}_iqr"] = iqr(vals)

        rows.append(row)

    out = pd.DataFrame(rows).sort_values(
        ["strategy", "plateau_rpm"]
    ).reset_index(drop=True)

    if not args.no_final_bin_smoothing and args.final_bin_smooth_window > 1:
        smooth_cols = [
            "p_dc_mean_W_mean",
            "p_cu_mean_W_mean",
            "phase_current_rms_A_mean",
            "omega_std_rpm_mean",
            "torque_ripple_rms_Nm_mean",
            "torque_ripple_robust_ptp_Nm_mean",
        ]

        for strategy, idx in out.groupby("strategy").groups.items():
            g = out.loc[idx].sort_values("plateau_rpm")
            sorted_idx = g.index

            for col in smooth_cols:
                if col not in out.columns:
                    continue
                out.loc[sorted_idx, col] = rolling_centered(
                    g[col], args.final_bin_smooth_window
                ).to_numpy()

    return out

# =============================================================================
# RUN PROCESSING
# =============================================================================

def process_one_run(job: dict, args):
    df = read_csv_checked(job["path"])
    has_js = detect_joulescope_status(df)

    df = process_torque(df, args)
    df, torque_baseline = apply_low_speed_torque_zero(df, args)
    df = compute_derived_quantities(df, args)

    df.insert(0, "run_id", f"{job['strategy']}_run{job['run_index']}")
    df.insert(0, "run_index", job["run_index"])
    df.insert(0, "source_file", str(job["path"]))
    df.insert(0, "strategy", job["strategy"])

    plateau_runs = plateau_stats_for_run(df, job, args)

    time = pd.to_numeric(df["host_time_s"], errors="coerce").to_numpy(dtype=float)
    pdc = pd.to_numeric(df["p_dc_in_calc_W"], errors="coerce").to_numpy(dtype=float)
    e_dc_j = integrate_trapezoid(time, pdc)

    rejected = int((df["torque_rejected_flag"] > 0).sum()) if "torque_rejected_flag" in df.columns else 0

    final_status = np.nan
    if "experiment_status" in df.columns:
        status = pd.to_numeric(df["experiment_status"], errors="coerce")
        if np.isfinite(status).any():
            final_status = int(round(float(status.dropna().iloc[-1])))

    fault_code = np.nan
    if "fault_code" in df.columns:
        fault = pd.to_numeric(df["fault_code"], errors="coerce")
        if np.isfinite(fault).any():
            fault_code = int(round(float(fault.dropna().iloc[-1])))

    run_summary = {
        "strategy": job["strategy"],
        "run_index": job["run_index"],
        "run_id": f"{job['strategy']}_run{job['run_index']}",
        "source_file": str(job["path"]),
        "raw_rows": int(len(df)),
        "plateau_points": int(len(plateau_runs)),
        "joulescope_present": bool(has_js),
        "torque_zero_baseline_Nm": torque_baseline,
        "torque_rejected_samples": rejected,
        "torque_rejected_percent": 100.0 * rejected / max(1, len(df)),
        "energy_dc_in_J": e_dc_j,
        "duration_s": float(np.nanmax(time) - np.nanmin(time)) if np.isfinite(time).any() else np.nan,
        "final_status": final_status,
        "fault_code": fault_code,
    }

    return df, plateau_runs, run_summary


def aggregate_run_summaries(run_summary: pd.DataFrame) -> pd.DataFrame:
    rows = []

    for strategy, g in run_summary.groupby("strategy"):
        row = {
            "strategy": strategy,
            "run_count": int(g["run_id"].nunique()),
        }

        for col in [
            "energy_dc_in_J",
            "duration_s",
            "torque_zero_baseline_Nm",
            "torque_rejected_percent",
        ]:
            vals = pd.to_numeric(g[col], errors="coerce").to_numpy(dtype=float)
            vals = vals[np.isfinite(vals)]

            row[f"{col}_mean"] = float(np.nanmean(vals)) if len(vals) else np.nan
            row[f"{col}_median"] = float(np.nanmedian(vals)) if len(vals) else np.nan
            row[f"{col}_std"] = (
                float(np.nanstd(vals, ddof=1))
                if len(vals) > 1
                else 0.0 if len(vals) == 1 else np.nan
            )
            row[f"{col}_iqr"] = iqr(vals)

        rows.append(row)

    return pd.DataFrame(rows)


def aggregate_timeseries(processed_ts: pd.DataFrame) -> pd.DataFrame:
    if processed_ts.empty:
        return pd.DataFrame()

    temp = processed_ts.copy()

    # Normalize each experimental trial to its own start time before combining
    # repeated runs. This prevents host-side start-time offsets from shifting
    # otherwise equivalent experiment profiles.
    temp["time_relative_s"] = (
        pd.to_numeric(temp["host_time_s"], errors="coerce")
        - temp.groupby("run_id")["host_time_s"].transform("min")
    )
    temp = temp[np.isfinite(temp["time_relative_s"])].copy()
    temp["time_relative_ms"] = np.round(temp["time_relative_s"] * 1000.0).astype(int)

    # Reduce every run to one value per millisecond first. The final standard
    # deviation is therefore computed across independent experimental trials,
    # rather than across a mixture of repeated samples and trials.
    per_run = (
        temp.groupby(["strategy", "run_id", "time_relative_ms"], as_index=False)
        .agg(
            p_dc_run_mean_W=("p_dc_in_calc_W", finite_mean),
            speed_command_run_mean_rpm=("speed_command_rpm", finite_mean),
            omega_run_mean_rpm=("omega_rpm", finite_mean),
        )
    )

    rows = []
    for (strategy, ms), g in per_run.groupby(["strategy", "time_relative_ms"]):
        rows.append({
            "strategy": strategy,
            "time_s": ms / 1000.0,
            "run_count": int(g["run_id"].nunique()),
            "p_dc_mean_W": finite_mean(g["p_dc_run_mean_W"]),
            "p_dc_std_W": finite_std(g["p_dc_run_mean_W"]),
            "speed_command_mean_rpm": finite_mean(g["speed_command_run_mean_rpm"]),
            "omega_mean_rpm": finite_mean(g["omega_run_mean_rpm"]),
        })

    return pd.DataFrame(rows).sort_values(
        ["strategy", "time_s"]
    ).reset_index(drop=True)

# =============================================================================
# PLOTTING
# =============================================================================

def setup_axes(title, xlabel, ylabel):
    fig, ax = plt.subplots(figsize=(7.0, 4.2))
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    return fig, ax


def save_figure(fig, outdir: Path, stem: str):
    outdir.mkdir(parents=True, exist_ok=True)
    png = outdir / f"{stem}.png"
    pdf = outdir / f"{stem}.pdf"
    fig.tight_layout()
    fig.savefig(png, dpi=300)
    fig.savefig(pdf)
    plt.close(fig)
    print(f"[FIG] {png}")
    print(f"[FIG] {pdf}")


def iter_strategy_groups(df):
    available = list(df["strategy"].dropna().unique())
    ordered = [s for s in STRATEGY_ORDER if s in available]
    ordered += [s for s in available if s not in ordered]
    for strategy in ordered:
        g = df[df["strategy"] == strategy]
        if not g.empty:
            yield strategy, g


def plot_time_series(ts_agg, outdir, y_col, std_col, title, ylabel, stem, show_std):
    if ts_agg is None or ts_agg.empty:
        print(f"[WARN] Skipping {stem}: no time-series data available.")
        return

    if "strategy" not in ts_agg.columns or "time_s" not in ts_agg.columns:
        print(f"[WARN] Skipping {stem}: missing required time-series columns.")
        return

    if y_col not in ts_agg.columns:
        print(f"[WARN] Skipping {stem}: missing column {y_col}.")
        return

    fig, ax = setup_axes(title, "Time (s)", ylabel)
    plotted = False

    for strategy, g in iter_strategy_groups(ts_agg):
        x = pd.to_numeric(g["time_s"], errors="coerce").to_numpy(dtype=float)
        y = pd.to_numeric(g[y_col], errors="coerce").to_numpy(dtype=float)

        mask = np.isfinite(x) & np.isfinite(y)
        if not np.any(mask):
            continue

        x = x[mask]
        y = y[mask]

        order = np.argsort(x)
        x = x[order]
        y = y[order]

        color = STRATEGY_COLORS.get(strategy)
        ax.plot(x, y, linewidth=1.8, label=strategy, color=color)
        plotted = True

        if show_std and std_col in g.columns:
            s_all = pd.to_numeric(g[std_col], errors="coerce").to_numpy(dtype=float)
            s = s_all[mask][order]
            finite_band = np.isfinite(s)
            if np.any(finite_band):
                ax.fill_between(
                    x[finite_band],
                    y[finite_band] - s[finite_band],
                    y[finite_band] + s[finite_band],
                    alpha=0.16,
                    color=color,
                )

    if not plotted:
        plt.close(fig)
        print(f"[WARN] Skipping {stem}: no finite data to plot.")
        return

    ax.legend()
    save_figure(fig, outdir, stem)


def plot_plateau_curve(agg, outdir, y_col, std_col, title, ylabel, stem, show_std, scale=1.0):
    if agg is None or agg.empty:
        print(f"[WARN] Skipping {stem}: no plateau data available.")
        return

    if "strategy" not in agg.columns:
        print(f"[WARN] Skipping {stem}: missing strategy column.")
        return

    if "plateau_rpm" not in agg.columns:
        print(f"[WARN] Skipping {stem}: missing plateau_rpm column.")
        return

    if y_col not in agg.columns:
        print(f"[WARN] Skipping {stem}: missing column {y_col}.")
        return

    fig, ax = setup_axes(title, "Commanded Speed (RPM)", ylabel)

    plotted = False

    for strategy, g in iter_strategy_groups(agg):
        x = pd.to_numeric(g["plateau_rpm"], errors="coerce").to_numpy(dtype=float)
        y = pd.to_numeric(g[y_col], errors="coerce").to_numpy(dtype=float)

        mask = np.isfinite(x) & np.isfinite(y)
        if not np.any(mask):
            print(f"[WARN] {stem}: no finite {y_col} data for {strategy}.")
            continue

        x = x[mask]
        y = y[mask] * scale

        order = np.argsort(x)
        x = x[order]
        y = y[order]
        color = STRATEGY_COLORS.get(strategy)

        if show_std and std_col in g.columns:
            s_all = pd.to_numeric(g[std_col], errors="coerce").to_numpy(dtype=float)
            s = s_all[mask][order] * scale
            finite = np.isfinite(s)
            if np.any(finite):
                ax.errorbar(
                    x[finite],
                    y[finite],
                    yerr=s[finite],
                    linewidth=2.1,
                    marker="o",
                    markersize=3.0,
                    capsize=3.0,
                    elinewidth=1.0,
                    label=strategy,
                    color=color,
                )
                if np.any(~finite):
                    ax.plot(x[~finite], y[~finite], linewidth=2.1, marker="o", markersize=3.0, color=color)
            else:
                ax.plot(x, y, linewidth=2.1, marker="o", markersize=3.0, label=strategy, color=color)
        else:
            ax.plot(x, y, linewidth=2.1, marker="o", markersize=3.0, label=strategy, color=color)

        plotted = True

    if not plotted:
        plt.close(fig)
        print(f"[WARN] Skipping {stem}: no finite data to plot.")
        return

    ax.legend()
    save_figure(fig, outdir, stem)


def plot_energy_bars(summary_agg, outdir):
    fig, ax = plt.subplots(figsize=(6.5, 4.0))

    ordered_rows = []
    for s in STRATEGY_ORDER:
        temp = summary_agg[summary_agg["strategy"] == s]
        if not temp.empty:
            ordered_rows.append(temp)
    rem = summary_agg[~summary_agg["strategy"].isin(STRATEGY_ORDER)]
    if not rem.empty:
        ordered_rows.append(rem)
    df = pd.concat(ordered_rows, ignore_index=True) if ordered_rows else summary_agg

    x = np.arange(len(df))
    y = pd.to_numeric(df["energy_dc_in_J_mean"], errors="coerce").to_numpy(dtype=float)
    yerr = pd.to_numeric(df["energy_dc_in_J_std"], errors="coerce").to_numpy(dtype=float)

    colors = [STRATEGY_COLORS.get(strategy) for strategy in df["strategy"]]
    ax.bar(x, y, yerr=yerr, capsize=4, color=colors, error_kw={"elinewidth": 1.0, "capthick": 1.0})
    ax.set_title("Total Experiment Energy Consumption")
    ax.set_xlabel("Commutation Strategy")
    ax.set_ylabel("Electrical Input Energy (J)")
    ax.set_xticks(x)
    ax.set_xticklabels(df["strategy"])
    ax.grid(True, axis="y", alpha=0.3)

    save_figure(fig, outdir, "total_energy_consumption")


def make_processing_settings(args):
    rows = [
        ("outdir", args.outdir),
        ("bin_width_rpm", args.bin_width_rpm),
        ("min_samples_per_bin", args.min_samples_per_bin),
        ("signed_torque", True),
        ("torque_filter_enabled", not args.no_torque_filter),
        ("max_torque_slew_mnm_s", args.max_torque_slew_mnm_s),
        ("hampel_window_s", args.hampel_window_s),
        ("hampel_n_sigma", args.hampel_n_sigma),
        ("mean_window_s", args.mean_window_s),
        ("final_bin_smoothing_enabled", not args.no_final_bin_smoothing),
        ("final_bin_smooth_window", args.final_bin_smooth_window),
        ("zero_torque_reference_enabled", not args.no_zero_torque_reference),
        ("zero_reference_max_rpm", args.zero_reference_max_rpm),
        ("phase_resistance_ohm", args.phase_resistance_ohm),
    ]
    return pd.DataFrame(rows, columns=["setting", "value"])


def write_outputs(outdir, processed_ts, ts_agg, plateau_runs, plateau_agg, run_summary, summary_agg, settings):
    outdir.mkdir(parents=True, exist_ok=True)

    outputs = {
        "power_ripple_timeseries_processed.csv": processed_ts,
        "power_ripple_timeseries_aggregate.csv": ts_agg,
        "power_ripple_plateau_runs.csv": plateau_runs,
        "power_ripple_plateau_aggregate_all.csv": plateau_agg,
        "power_ripple_run_summary.csv": run_summary,
        "power_ripple_experiment_summary.csv": summary_agg,
        "power_ripple_processing_settings.csv": settings,
    }

    for filename, df in outputs.items():
        path = outdir / filename
        df.to_csv(path, index=False)
        print(f"[CSV] {path}")

    if not plateau_agg.empty:
        for strategy, g in plateau_agg.groupby("strategy"):
            safe = strategy.lower().replace(" ", "_")
            path = outdir / f"power_ripple_plateau_aggregate_{safe}.csv"
            g.to_csv(path, index=False)
            print(f"[CSV] {path}")


# =============================================================================
# MAIN
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description="Process, aggregate, and plot OreSat PowerRippleTest CSVs.")

    parser.add_argument("--trap", nargs="+", help="One or more trapezoidal PowerRipple CSVs")
    parser.add_argument("--sine", nargs="+", help="One or more sinusoidal PowerRipple CSVs")
    parser.add_argument("--foc", nargs="+", help="One or more FOC PowerRipple CSVs")

    parser.add_argument("--outdir", default=DEFAULT_OUTDIR)

    parser.add_argument("--bin-width-rpm", type=float, default=DEFAULT_BIN_WIDTH_RPM)
    parser.add_argument("--min-samples-per-bin", type=int, default=DEFAULT_MIN_SAMPLES_PER_BIN)

    parser.add_argument("--no-torque-filter", action="store_true")
    parser.add_argument("--max-torque-slew-mnm-s", type=float, default=DEFAULT_MAX_TORQUE_SLEW_MNM_S)
    parser.add_argument("--hampel-window-s", type=float, default=DEFAULT_HAMPEL_WINDOW_S)
    parser.add_argument("--hampel-n-sigma", type=float, default=DEFAULT_HAMPEL_N_SIGMA)
    parser.add_argument("--mean-window-s", type=float, default=DEFAULT_MEAN_WINDOW_S)

    parser.add_argument("--final-bin-smooth-window", type=int, default=DEFAULT_FINAL_BIN_SMOOTH_WINDOW)
    parser.add_argument("--no-final-bin-smoothing", action="store_true")

    parser.add_argument(
        "--enable-zero-torque-reference",
        dest="no_zero_torque_reference",
        action="store_false",
        help="Apply an additional low-speed torque baseline correction. Disabled by default because the LabJack capture is already tared.",
    )
    parser.add_argument(
        "--no-zero-torque-reference",
        dest="no_zero_torque_reference",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    parser.set_defaults(no_zero_torque_reference=True)
    parser.add_argument("--zero-reference-max-rpm", type=float, default=DEFAULT_ZERO_REFERENCE_MAX_RPM)

    parser.add_argument("--phase-resistance-ohm", type=float, default=DEFAULT_PHASE_RESISTANCE_OHM)
    parser.add_argument(
        "--no-show-std",
        dest="show_std",
        action="store_false",
        help="Disable the default ±1 standard-deviation uncertainty display.",
    )
    parser.set_defaults(show_std=True)

    args = parser.parse_args()

    jobs = collect_inputs(args)

    print("[INPUT] Processing PowerRipple CSVs:")
    for job in jobs:
        print(f"  {job['strategy']} run {job['run_index']}: {job['path']}")

    processed_frames = []
    plateau_frames = []
    summaries = []

    for job in jobs:
        processed, plateau, summary = process_one_run(job, args)

        processed_frames.append(processed)
        if not plateau.empty:
            plateau_frames.append(plateau)
        summaries.append(summary)

        print(
            f"[RUN] {summary['strategy']} run {summary['run_index']}: "
            f"rows={summary['raw_rows']} "
            f"plateaus={summary['plateau_points']} "
            f"torque_rejected={summary['torque_rejected_samples']} "
            f"({summary['torque_rejected_percent']:.2f}%) "
            f"E_in={summary['energy_dc_in_J']:.3f} J "
            f"status={summary['final_status']} fault={summary['fault_code']}"
        )

    processed_ts = pd.concat(processed_frames, ignore_index=True) if processed_frames else pd.DataFrame()
    plateau_runs = pd.concat(plateau_frames, ignore_index=True) if plateau_frames else pd.DataFrame()
    run_summary = pd.DataFrame(summaries)

    ts_agg = aggregate_timeseries(processed_ts)
    plateau_agg = aggregate_plateaus(plateau_runs, args)
    summary_agg = aggregate_run_summaries(run_summary)
    settings = make_processing_settings(args)

    outdir = Path(args.outdir)

    has_power_data = has_usable_power_data(processed_ts)

    if not has_power_data:
        print("[WARN] No usable Joulescope/input-current data found.")
        print("[WARN] Skipping electrical input-power, input-energy, and input-power-versus-speed plots.")

    if has_power_data:
        plot_time_series(
            ts_agg,
            outdir,
            y_col="p_dc_mean_W",
            std_col="p_dc_std_W",
            title="Electrical Input Power Versus Time",
            ylabel="Electrical Input Power (W)",
            stem="power_input_vs_time",
            show_std=args.show_std,
        )

        plot_plateau_curve(
            plateau_agg,
            outdir,
            y_col="p_dc_mean_W_mean",
            std_col="p_dc_mean_W_std",
            title="Steady-State Electrical Input Power Comparison",
            ylabel="Electrical Input Power (W)",
            stem="power_input_vs_rpm",
            show_std=args.show_std,
        )

        plot_energy_bars(summary_agg, outdir)

    plot_plateau_curve(
        plateau_agg,
        outdir,
        y_col="phase_current_rms_A_mean",
        std_col="phase_current_rms_A_std",
        title="Phase Current RMS Comparison",
        ylabel="Phase Current RMS (A)",
        stem="phase_current_rms_vs_rpm",
        show_std=args.show_std,
    )

    plot_plateau_curve(
        plateau_agg,
        outdir,
        y_col="p_cu_mean_W_mean",
        std_col="p_cu_mean_W_std",
        title="Estimated Copper Loss Comparison",
        ylabel="Copper Loss (W)",
        stem="copper_loss_vs_rpm",
        show_std=args.show_std,
    )

    plot_plateau_curve(
        plateau_agg,
        outdir,
        y_col="omega_std_rpm_mean",
        std_col="omega_std_rpm_std",
        title="Velocity Stability Comparison",
        ylabel="Speed Standard Deviation (RPM)",
        stem="velocity_stability_vs_rpm",
        show_std=args.show_std,
    )

    plot_plateau_curve(
        plateau_agg,
        outdir,
        y_col="torque_ripple_rms_Nm_mean",
        std_col="torque_ripple_rms_Nm_std",
        title="Measured Reaction-Torque RMS Variation",
        ylabel="Reaction-Torque RMS Variation (µN·m)",
        stem="torque_ripple_rms_vs_rpm",
        show_std=args.show_std,
        scale=1.0e6,
    )

    plot_plateau_curve(
        plateau_agg,
        outdir,
        y_col="torque_ripple_robust_ptp_Nm_mean",
        std_col="torque_ripple_robust_ptp_Nm_std",
        title="Measured Reaction-Torque Robust Amplitude",
        ylabel="95th–5th Percentile Reaction-Torque Span (µN·m)",
        stem="torque_ripple_robust_ptp_vs_rpm",
        show_std=args.show_std,
        scale=1.0e6,
    )

    write_outputs(
        outdir=outdir,
        processed_ts=processed_ts,
        ts_agg=ts_agg,
        plateau_runs=plateau_runs,
        plateau_agg=plateau_agg,
        run_summary=run_summary,
        summary_agg=summary_agg,
        settings=settings,
    )

    print("[DONE] PowerRipple processing, aggregation, and plotting complete.")


if __name__ == "__main__":
    main()