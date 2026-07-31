"""
DynamicResponseResults.py

One-stop dynamic-response processing, aggregation, and plotting script for
OreSat reaction wheel thesis tests.

Recommended location:
    DynamicResponseTest/DynamicResponseResults.py

Purpose:
    Feed this script one or more raw DynamicResponse CSVs for each commutation
    method. It extracts the positive and negative velocity steps from every run,
    aligns all runs at t = 0, averages the responses, computes response metrics,
    and generates thesis-ready plots.

Primary figures:
    dynamic_positive_velocity_step.png/.pdf
    dynamic_negative_velocity_step.png/.pdf
    dynamic_positive_step_torque.png/.pdf
    dynamic_negative_step_torque.png/.pdf
    dynamic_rise_time_comparison.png/.pdf
    dynamic_fall_time_comparison.png/.pdf
    dynamic_positive_overshoot_comparison.png/.pdf
    dynamic_negative_overshoot_comparison.png/.pdf
    dynamic_positive_undershoot_comparison.png/.pdf
    dynamic_negative_undershoot_comparison.png/.pdf
    dynamic_positive_settling_time_comparison.png/.pdf
    dynamic_negative_settling_time_comparison.png/.pdf

Important behavior:
    - Overshoot is reported as 0 when no overshoot occurs.
    - Undershoot is reported as 0 when the response does not initially move
      opposite the commanded step direction.
    - Settling time is marked Did Not Settle when the response never remains
      inside the settling band long enough.
    - Did Not Settle bars are shown at the analysis window limit and labeled DNS.
    - Velocity and torque step traces are averaged across repeated runs.

Example:
    python DynamicResponseTest\DynamicResponseResults.py --trap DynamicResponseTest\V2\dynamic_response_trap_raw.csv --sine DynamicResponseTest\V2\dynamic_response_sine_raw.csv --foc DynamicResponseTest\V2\dynamic_response_foc_raw.csv
"""

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


DEFAULT_OUTDIR = "Figures"

DEFAULT_PRE_WINDOW_S = 1.0
DEFAULT_POST_WINDOW_S = 8.0

DEFAULT_SETTLING_BAND_PERCENT = 2.0
DEFAULT_SETTLING_HOLD_S = 0.5

DEFAULT_VELOCITY_MEAN_WINDOW_S = 0.05

# Dynamic torque can legitimately change quickly, so these defaults are looser
# than the torque-speed test.
DEFAULT_MAX_TORQUE_SLEW_UNM_S = 500000.0
DEFAULT_HAMPEL_WINDOW_S = 0.25
DEFAULT_HAMPEL_N_SIGMA = 4.0
DEFAULT_TORQUE_MEAN_WINDOW_S = 0.20

DEFAULT_POSITIVE_LOW_RPM = 2000.0
DEFAULT_POSITIVE_HIGH_RPM = 6000.0
DEFAULT_NEGATIVE_HIGH_RPM = 6000.0
DEFAULT_NEGATIVE_LOW_RPM = 2000.0

STRATEGY_INPUTS = {
    "Trapezoidal": "trap",
    "Sinusoidal": "sine",
    "FOC": "foc",
}

STRATEGY_ORDER = ["Trapezoidal", "Sinusoidal", "FOC"]

STRATEGY_COLORS = {
    "Trapezoidal": "#1f77b4",
    "Sinusoidal": "#ff7f0e",
    "FOC": "#2ca02c",
}

POSITIVE = "positive"
NEGATIVE = "negative"


def read_csv_checked(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)

    required = ["host_time_s", "omega_rpm", "speed_command_rpm", "torque_Nm"]
    for column in required:
        if column not in df.columns:
            raise ValueError(f"{path} is missing required column: {column}")

    df = df.copy()

    numeric_columns = [
        "host_time_s", "omega_rpm", "speed_command_rpm", "torque_Nm",
        "torque_sensor_V", "torque_sensor_tare_V", "torque_sensor_delta_V",
        "experiment_status", "fault_code", "test_state",
        "js_voltage_V", "js_current_A", "js_power_W",
    ]

    for column in numeric_columns:
        if column in df.columns:
            df[column] = pd.to_numeric(df[column], errors="coerce")

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
        raise ValueError("No input CSV files provided.")

    return jobs


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
    if not np.isfinite(window_s) or window_s <= 0:
        return 1
    if not np.isfinite(dt_s) or dt_s <= 0:
        return 1
    return max(1, int(round(window_s / dt_s)))


def rolling_mean_by_time(values: pd.Series, dt_s: float, window_s: float) -> pd.Series:
    y = pd.to_numeric(values, errors="coerce").astype(float)
    n = make_odd(window_seconds_to_samples(window_s, dt_s))
    if n <= 1:
        return y
    out = y.rolling(window=n, center=True, min_periods=max(1, n // 3)).mean()
    return out.interpolate(limit_direction="both").bfill().ffill()


def iqr(values) -> float:
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if len(values) == 0:
        return np.nan
    q75, q25 = np.nanpercentile(values, [75, 25])
    return float(q75 - q25)


def reject_slew_spikes(values: pd.Series, time_s: pd.Series, max_slew_unm_s: float) -> tuple[pd.Series, pd.Series]:
    y = pd.to_numeric(values, errors="coerce").astype(float).copy()
    t = pd.to_numeric(time_s, errors="coerce").astype(float)
    flags = pd.Series(False, index=y.index)

    if not np.isfinite(max_slew_unm_s) or max_slew_unm_s <= 0:
        return y, flags

    max_slew_nm_s = max_slew_unm_s / 1.0e6

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
        if not np.isfinite(dt) or dt <= 0:
            last_t = ti
            last_y = yi
            continue

        if abs(yi - last_y) > max_slew_nm_s * dt:
            flags.loc[idx] = True
            y.loc[idx] = np.nan
            continue

        last_t = ti
        last_y = yi

    y = y.interpolate(limit_direction="both").bfill().ffill()
    return y, flags


def hampel_filter(values: pd.Series, dt_s: float, window_s: float, n_sigma: float) -> tuple[pd.Series, pd.Series]:
    y = pd.to_numeric(values, errors="coerce").astype(float)
    n = make_odd(window_seconds_to_samples(window_s, dt_s))
    if n <= 1:
        return y, pd.Series(False, index=y.index)

    med = y.rolling(window=n, center=True, min_periods=max(1, n // 3)).median()
    dev = (y - med).abs()
    mad = dev.rolling(window=n, center=True, min_periods=max(1, n // 3)).median()
    sigma = 1.4826 * mad
    threshold = n_sigma * sigma

    flags = (dev > threshold) & np.isfinite(dev) & np.isfinite(threshold) & (threshold > 0)
    clean = y.copy()
    clean.loc[flags] = med.loc[flags]
    clean = clean.interpolate(limit_direction="both").bfill().ffill()

    return clean, flags


def process_torque(df: pd.DataFrame, args) -> pd.DataFrame:
    out = df.copy()
    dt = estimate_sample_period_s(out)
    out["torque_raw_Nm"] = pd.to_numeric(out["torque_Nm"], errors="coerce").astype(float)

    if args.no_torque_filter:
        out["torque_filtered_Nm"] = out["torque_raw_Nm"]
        out["torque_rejected_flag"] = 0
        return out

    slew, slew_flags = reject_slew_spikes(
        out["torque_raw_Nm"],
        out["host_time_s"],
        args.max_torque_slew_unm_s,
    )

    hampel, hampel_flags = hampel_filter(
        slew,
        dt,
        args.hampel_window_s,
        args.hampel_n_sigma,
    )

    out["torque_filtered_Nm"] = rolling_mean_by_time(
        hampel,
        dt,
        args.torque_mean_window_s,
    )

    out["torque_rejected_flag"] = (slew_flags | hampel_flags).astype(int)
    return out


def crossing_time(t: np.ndarray, y: np.ndarray, threshold: float, rising: bool) -> float:
    if rising:
        idx = np.where(y >= threshold)[0]
    else:
        idx = np.where(y <= threshold)[0]

    if len(idx) == 0:
        return np.nan

    i = int(idx[0])
    if i == 0:
        return float(t[i])

    t0, t1 = t[i - 1], t[i]
    y0, y1 = y[i - 1], y[i]

    if not all(np.isfinite(v) for v in [t0, t1, y0, y1]) or y1 == y0:
        return float(t1)

    frac = (threshold - y0) / (y1 - y0)
    frac = max(0.0, min(1.0, frac))
    return float(t0 + frac * (t1 - t0))


def detect_steps_by_command_levels(df: pd.DataFrame, args) -> dict:
    t = df["host_time_s"].to_numpy(dtype=float)
    cmd = df["speed_command_rpm"].to_numpy(dtype=float)

    pos_mid = 0.5 * (args.positive_low_rpm + args.positive_high_rpm)
    neg_mid = 0.5 * (args.negative_high_rpm + args.negative_low_rpm)

    pos_candidates = np.where((cmd[:-1] < pos_mid) & (cmd[1:] >= pos_mid))[0] + 1
    if len(pos_candidates) == 0:
        # Fallback: largest positive command slope.
        dcmd = np.diff(cmd, prepend=cmd[0])
        pos_candidates = np.array([int(np.nanargmax(dcmd))])

    pos_idx = int(pos_candidates[0])

    neg_candidates = np.where((cmd[:-1] > neg_mid) & (cmd[1:] <= neg_mid))[0] + 1
    neg_candidates = neg_candidates[neg_candidates > pos_idx]
    if len(neg_candidates) == 0:
        dcmd = np.diff(cmd, prepend=cmd[0])
        later = np.arange(len(dcmd))[np.arange(len(dcmd)) > pos_idx]
        if len(later) == 0:
            raise ValueError("Could not detect negative speed step.")
        neg_idx = int(later[np.nanargmin(dcmd[later])])
    else:
        neg_idx = int(neg_candidates[0])

    return {
        POSITIVE: {
            "index": pos_idx,
            "time_s": float(t[pos_idx]),
            "command_before_rpm": float(args.positive_low_rpm),
            "command_after_rpm": float(args.positive_high_rpm),
            "step_delta_rpm": float(args.positive_high_rpm - args.positive_low_rpm),
        },
        NEGATIVE: {
            "index": neg_idx,
            "time_s": float(t[neg_idx]),
            "command_before_rpm": float(args.negative_high_rpm),
            "command_after_rpm": float(args.negative_low_rpm),
            "step_delta_rpm": float(args.negative_low_rpm - args.negative_high_rpm),
        },
    }


def extract_window(df: pd.DataFrame, event: dict, label: str, args) -> pd.DataFrame:
    t0 = event["time_s"]
    win = df[(df["host_time_s"] >= t0 - args.pre_window_s) &
             (df["host_time_s"] <= t0 + args.post_window_s)].copy()

    win["step_label"] = label
    win["step_time_s"] = t0
    win["time_relative_s"] = win["host_time_s"] - t0
    win["command_before_rpm"] = event["command_before_rpm"]
    win["command_after_rpm"] = event["command_after_rpm"]
    win["step_delta_rpm"] = event["step_delta_rpm"]

    return win.reset_index(drop=True)


def settling_time(t: np.ndarray, y: np.ndarray, final_value: float, band: float, hold_s: float) -> float:
    inside = np.abs(y - final_value) <= band
    for i in range(len(t)):
        if not inside[i]:
            continue
        end_t = t[i] + hold_s
        j = np.searchsorted(t, end_t)
        if j >= len(t):
            continue
        if np.all(inside[i:j + 1]):
            return float(t[i])
    return np.nan


def compute_metrics(win: pd.DataFrame, label: str, args) -> dict:
    dt = estimate_sample_period_s(win)

    t = win["time_relative_s"].to_numpy(dtype=float)
    omega_raw = pd.to_numeric(win["omega_rpm"], errors="coerce").astype(float)
    omega = rolling_mean_by_time(omega_raw, dt, args.velocity_mean_window_s).to_numpy(dtype=float)

    before = float(win["command_before_rpm"].iloc[0])
    after = float(win["command_after_rpm"].iloc[0])
    delta = after - before
    mag = abs(delta)

    post = t >= 0
    t_post = t[post]
    y_post = omega[post]

    band = mag * args.settling_band_percent / 100.0

    overshoot_rpm = np.nan
    undershoot_rpm = np.nan

    if delta > 0:
        t10 = crossing_time(t_post, y_post, before + 0.10 * delta, rising=True)
        t90 = crossing_time(t_post, y_post, before + 0.90 * delta, rising=True)
        rise_time = t90 - t10 if np.isfinite(t10) and np.isfinite(t90) else np.nan
        fall_time = np.nan

        peak = float(np.nanmax(y_post)) if len(y_post) else np.nan
        valley = float(np.nanmin(y_post)) if len(y_post) else np.nan

        # Overshoot: excursion beyond the final command in the commanded direction.
        overshoot_rpm = max(0.0, peak - after) if np.isfinite(peak) else np.nan

        # Undershoot: wrong-direction excursion below the pre-step value.
        # This avoids counting the normal rise from "before" to "after" as undershoot.
        undershoot_rpm = max(0.0, before - valley) if np.isfinite(valley) else np.nan
    else:
        # During a negative step, fall time is from 90% to 10% of the step.
        t90 = crossing_time(t_post, y_post, before + 0.10 * delta, rising=False)
        t10 = crossing_time(t_post, y_post, before + 0.90 * delta, rising=False)
        fall_time = t10 - t90 if np.isfinite(t10) and np.isfinite(t90) else np.nan
        rise_time = np.nan

        peak = float(np.nanmax(y_post)) if len(y_post) else np.nan
        valley = float(np.nanmin(y_post)) if len(y_post) else np.nan

        # Overshoot: excursion beyond the final command in the commanded direction.
        overshoot_rpm = max(0.0, after - valley) if np.isfinite(valley) else np.nan

        # Undershoot: wrong-direction excursion above the pre-step value.
        undershoot_rpm = max(0.0, peak - before) if np.isfinite(peak) else np.nan

    overshoot_percent = 100.0 * overshoot_rpm / mag if np.isfinite(overshoot_rpm) and mag > 0 else np.nan
    undershoot_percent = 100.0 * undershoot_rpm / mag if np.isfinite(undershoot_rpm) and mag > 0 else np.nan

    ts = settling_time(t_post, y_post, after, band, args.settling_hold_s)
    settled = bool(np.isfinite(ts))
    settling_plot_s = ts if settled else args.post_window_s

    torque_post = pd.to_numeric(win.loc[post, "torque_filtered_Nm"], errors="coerce")
    peak_abs_torque_nm = float(np.nanmax(np.abs(torque_post))) if len(torque_post) else np.nan

    return {
        "step_label": label,
        "command_before_rpm": before,
        "command_after_rpm": after,
        "step_delta_rpm": delta,
        "step_magnitude_rpm": mag,
        "rise_time_s": rise_time,
        "fall_time_s": fall_time,
        "overshoot_rpm": overshoot_rpm,
        "overshoot_percent": overshoot_percent,
        "undershoot_rpm": undershoot_rpm,
        "undershoot_percent": undershoot_percent,
        "settling_time_s": ts,
        "settling_time_plot_s": settling_plot_s,
        "settled": settled,
        "settling_band_percent": args.settling_band_percent,
        "settling_band_rpm": band,
        "peak_abs_torque_Nm": peak_abs_torque_nm,
        "peak_abs_torque_uNm": peak_abs_torque_nm * 1.0e6 if np.isfinite(peak_abs_torque_nm) else np.nan,
    }


def process_one_run(job: dict, args):
    df = read_csv_checked(job["path"])
    df = process_torque(df, args)

    events = detect_steps_by_command_levels(df, args)

    windows = []
    metrics = []

    for label in [POSITIVE, NEGATIVE]:
        win = extract_window(df, events[label], label, args)
        run_id = f"{job['strategy']}_run{job['run_index']}"

        win.insert(0, "run_id", run_id)
        win.insert(0, "run_index", job["run_index"])
        win.insert(0, "source_file", str(job["path"]))
        win.insert(0, "strategy", job["strategy"])

        m = compute_metrics(win, label, args)
        m.update({
            "strategy": job["strategy"],
            "run_index": job["run_index"],
            "run_id": run_id,
            "source_file": str(job["path"]),
        })

        windows.append(win)
        metrics.append(m)

    status = np.nan
    if "experiment_status" in df.columns:
        s = pd.to_numeric(df["experiment_status"], errors="coerce")
        if np.isfinite(s).any():
            status = int(round(float(s.dropna().iloc[-1])))

    fault = np.nan
    if "fault_code" in df.columns:
        f = pd.to_numeric(df["fault_code"], errors="coerce")
        if np.isfinite(f).any():
            fault = int(round(float(f.dropna().iloc[-1])))

    rejected = int((df["torque_rejected_flag"] > 0).sum()) if "torque_rejected_flag" in df.columns else 0

    summary = {
        "strategy": job["strategy"],
        "run_index": job["run_index"],
        "source_file": str(job["path"]),
        "raw_rows": len(df),
        "torque_rejected_samples": rejected,
        "torque_rejected_percent": 100.0 * rejected / max(1, len(df)),
        "final_status": status,
        "fault_code": fault,
    }

    return pd.concat(windows, ignore_index=True), pd.DataFrame(metrics), summary


def aggregate_timeseries(raw_ts: pd.DataFrame) -> pd.DataFrame:
    if raw_ts.empty:
        return pd.DataFrame()

    temp = raw_ts.copy()
    temp["time_relative_ms"] = np.round(temp["time_relative_s"] * 1000.0).astype(int)

    rows = []
    for (strategy, label, ms), g in temp.groupby(["strategy", "step_label", "time_relative_ms"]):
        rows.append({
            "strategy": strategy,
            "step_label": label,
            "time_relative_s": ms / 1000.0,
            "run_count": int(g["run_id"].nunique()),
            "omega_mean_rpm": float(np.nanmean(g["omega_rpm"])),
            "omega_std_rpm": float(np.nanstd(g["omega_rpm"], ddof=1)) if len(g) > 1 else 0.0,
            "command_mean_rpm": float(np.nanmean(g["speed_command_rpm"])),
            "torque_mean_Nm": float(np.nanmean(g["torque_filtered_Nm"])),
            "torque_std_Nm": float(np.nanstd(g["torque_filtered_Nm"], ddof=1)) if len(g) > 1 else 0.0,
            "torque_mean_uNm": float(np.nanmean(g["torque_filtered_Nm"]) * 1.0e6),
            "torque_std_uNm": float((np.nanstd(g["torque_filtered_Nm"], ddof=1) if len(g) > 1 else 0.0) * 1.0e6),
        })

    return pd.DataFrame(rows).sort_values(["strategy", "step_label", "time_relative_s"]).reset_index(drop=True)


def summarize_metrics(metrics: pd.DataFrame) -> pd.DataFrame:
    rows = []
    metric_cols = [
        "rise_time_s",
        "fall_time_s",
        "overshoot_percent",
        "undershoot_percent",
        "settling_time_s",
        "settling_time_plot_s",
        "peak_abs_torque_uNm",
    ]

    for (strategy, label), g in metrics.groupby(["strategy", "step_label"]):
        row = {
            "strategy": strategy,
            "step_label": label,
            "run_count": int(g["run_id"].nunique()),
            "settled_run_count": int(g["settled"].sum()) if "settled" in g.columns else 0,
            "did_not_settle_count": int((~g["settled"]).sum()) if "settled" in g.columns else 0,
        }

        for col in metric_cols:
            vals = pd.to_numeric(g[col], errors="coerce").to_numpy(dtype=float)
            vals = vals[np.isfinite(vals)]
            row[f"{col}_mean"] = float(np.nanmean(vals)) if len(vals) else np.nan
            row[f"{col}_median"] = float(np.nanmedian(vals)) if len(vals) else np.nan
            row[f"{col}_std"] = float(np.nanstd(vals, ddof=1)) if len(vals) > 1 else 0.0 if len(vals) == 1 else np.nan
            row[f"{col}_iqr"] = iqr(vals)

        rows.append(row)

    return pd.DataFrame(rows)


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


def plot_velocity(agg, raw, label, outdir, args):
    title = "Positive Velocity Step Response" if label == POSITIVE else "Negative Velocity Step Response"
    stem = "dynamic_positive_velocity_step" if label == POSITIVE else "dynamic_negative_velocity_step"

    fig, ax = setup_axes(title, "Time After Step (s)", "Wheel Speed (RPM)")

    if args.show_run_traces:
        for (strategy, _), g in raw[raw["step_label"] == label].groupby(["strategy", "run_id"]):
            color = STRATEGY_COLORS.get(strategy)
            ax.plot(
                g["time_relative_s"],
                g["omega_rpm"],
                linewidth=0.6,
                alpha=0.18,
                color=color,
            )

    command_plotted = False
    for strategy, g in iter_strategy_groups(agg[agg["step_label"] == label]):
        x = g["time_relative_s"].to_numpy(dtype=float)
        y = g["omega_mean_rpm"].to_numpy(dtype=float)
        cmd = g["command_mean_rpm"].to_numpy(dtype=float)
        color = STRATEGY_COLORS.get(strategy)

        ax.plot(x, y, linewidth=2.2, label=strategy, color=color)

        if not command_plotted:
            ax.plot(
                x,
                cmd,
                linewidth=1.0,
                linestyle="--",
                alpha=0.65,
                color="black",
                label="Command",
            )
            command_plotted = True

        if not args.skip_std_band:
            s = g["omega_std_rpm"].to_numpy(dtype=float)
            ax.fill_between(
                x,
                y - s,
                y + s,
                alpha=0.15,
                color=color,
                linewidth=0.0,
            )

    ax.axvline(0.0, linewidth=0.8, alpha=0.6, color="black")
    ax.legend(title="Commutation Strategy")
    save_figure(fig, outdir, stem)


def plot_torque(agg, raw, label, outdir, args):
    title = "Reaction Torque During Positive Step" if label == POSITIVE else "Reaction Torque During Negative Step"
    stem = "dynamic_positive_step_torque" if label == POSITIVE else "dynamic_negative_step_torque"

    fig, ax = setup_axes(title, "Time After Step (s)", "Reaction Torque (µN·m)")

    if args.show_run_traces:
        for (strategy, _), g in raw[raw["step_label"] == label].groupby(["strategy", "run_id"]):
            color = STRATEGY_COLORS.get(strategy)
            ax.plot(
                g["time_relative_s"],
                g["torque_filtered_Nm"] * 1.0e6,
                linewidth=0.6,
                alpha=0.18,
                color=color,
            )

    for strategy, g in iter_strategy_groups(agg[agg["step_label"] == label]):
        x = g["time_relative_s"].to_numpy(dtype=float)
        y = g["torque_mean_uNm"].to_numpy(dtype=float)
        color = STRATEGY_COLORS.get(strategy)

        ax.plot(x, y, linewidth=2.2, label=strategy, color=color)

        if not args.skip_std_band:
            s = g["torque_std_uNm"].to_numpy(dtype=float)
            ax.fill_between(
                x,
                y - s,
                y + s,
                alpha=0.15,
                color=color,
                linewidth=0.0,
            )

    ax.axvline(0.0, linewidth=0.8, alpha=0.6, color="black")
    ax.axhline(0.0, linewidth=0.8, alpha=0.5, color="black")
    ax.legend(title="Commutation Strategy")
    save_figure(fig, outdir, stem)


def ordered_summary(summary, label):
    g = summary[summary["step_label"] == label].copy()
    rows = []
    for s in STRATEGY_ORDER:
        temp = g[g["strategy"] == s]
        if not temp.empty:
            rows.append(temp)
    rem = g[~g["strategy"].isin(STRATEGY_ORDER)]
    if not rem.empty:
        rows.append(rem)
    return pd.concat(rows, ignore_index=True) if rows else g


def plot_metric(summary, label, metric_base, title, ylabel, outdir, stem, dns=False):
    g = ordered_summary(summary, label)
    mean_col = f"{metric_base}_mean"
    std_col = f"{metric_base}_std"

    if g.empty or mean_col not in g.columns:
        return

    fig, ax = plt.subplots(figsize=(6.5, 4.0))

    x = np.arange(len(g))
    y = pd.to_numeric(g[mean_col], errors="coerce").to_numpy(dtype=float)
    yerr = pd.to_numeric(g[std_col], errors="coerce").to_numpy(dtype=float) if std_col in g.columns else None

    bar_colors = [STRATEGY_COLORS.get(strategy) for strategy in g["strategy"]]
    ax.bar(x, y, yerr=yerr, capsize=4, color=bar_colors)
    ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.set_xlabel("Commutation Strategy")
    ax.set_xticks(x)
    ax.set_xticklabels(g["strategy"])
    ax.grid(True, axis="y", alpha=0.3)

    if dns and "did_not_settle_count" in g.columns:
        for i, row in g.reset_index(drop=True).iterrows():
            if int(row["did_not_settle_count"]) > 0:
                ax.text(i, y[i], "DNS", ha="center", va="bottom", fontsize=9)

    save_figure(fig, outdir, stem)


def write_outputs(outdir, raw_ts, agg_ts, metrics, summary, run_summary, settings):
    outdir.mkdir(parents=True, exist_ok=True)
    outputs = {
        "dynamic_response_step_timeseries.csv": raw_ts,
        "dynamic_response_step_timeseries_aggregate.csv": agg_ts,
        "dynamic_response_metrics_by_run.csv": metrics,
        "dynamic_response_metrics_summary.csv": summary,
        "dynamic_response_run_summary.csv": run_summary,
        "dynamic_response_processing_settings.csv": settings,
    }

    for name, df in outputs.items():
        path = outdir / name
        df.to_csv(path, index=False)
        print(f"[CSV] {path}")


def settings_table(args):
    return pd.DataFrame([
        ("pre_window_s", args.pre_window_s),
        ("post_window_s", args.post_window_s),
        ("settling_band_percent", args.settling_band_percent),
        ("settling_hold_s", args.settling_hold_s),
        ("velocity_mean_window_s", args.velocity_mean_window_s),
        ("torque_filter_enabled", not args.no_torque_filter),
        ("max_torque_slew_unm_s", args.max_torque_slew_unm_s),
        ("hampel_window_s", args.hampel_window_s),
        ("hampel_n_sigma", args.hampel_n_sigma),
        ("torque_mean_window_s", args.torque_mean_window_s),
        ("positive_low_rpm", args.positive_low_rpm),
        ("positive_high_rpm", args.positive_high_rpm),
        ("negative_high_rpm", args.negative_high_rpm),
        ("negative_low_rpm", args.negative_low_rpm),
        ("show_standard_deviation", not args.skip_std_band),
    ], columns=["setting", "value"])


def main():
    parser = argparse.ArgumentParser(description="Process, aggregate, and plot OreSat dynamic response CSVs.")

    parser.add_argument("--trap", nargs="+")
    parser.add_argument("--sine", nargs="+")
    parser.add_argument("--foc", nargs="+")
    parser.add_argument("--outdir", default=DEFAULT_OUTDIR)

    parser.add_argument("--pre-window-s", type=float, default=DEFAULT_PRE_WINDOW_S)
    parser.add_argument("--post-window-s", type=float, default=DEFAULT_POST_WINDOW_S)

    parser.add_argument("--settling-band-percent", type=float, default=DEFAULT_SETTLING_BAND_PERCENT)
    parser.add_argument("--settling-hold-s", type=float, default=DEFAULT_SETTLING_HOLD_S)

    parser.add_argument("--velocity-mean-window-s", type=float, default=DEFAULT_VELOCITY_MEAN_WINDOW_S)

    parser.add_argument("--no-torque-filter", action="store_true")
    parser.add_argument("--max-torque-slew-unm-s", type=float, default=DEFAULT_MAX_TORQUE_SLEW_UNM_S)
    parser.add_argument("--hampel-window-s", type=float, default=DEFAULT_HAMPEL_WINDOW_S)
    parser.add_argument("--hampel-n-sigma", type=float, default=DEFAULT_HAMPEL_N_SIGMA)
    parser.add_argument("--torque-mean-window-s", type=float, default=DEFAULT_TORQUE_MEAN_WINDOW_S)

    parser.add_argument("--positive-low-rpm", type=float, default=DEFAULT_POSITIVE_LOW_RPM)
    parser.add_argument("--positive-high-rpm", type=float, default=DEFAULT_POSITIVE_HIGH_RPM)
    parser.add_argument("--negative-high-rpm", type=float, default=DEFAULT_NEGATIVE_HIGH_RPM)
    parser.add_argument("--negative-low-rpm", type=float, default=DEFAULT_NEGATIVE_LOW_RPM)

    parser.add_argument("--show-run-traces", action="store_true")
    parser.add_argument("--skip-std-band", action="store_true")

    args = parser.parse_args()

    jobs = collect_inputs(args)

    ts_frames = []
    metric_frames = []
    run_summaries = []

    print("[INPUT] Processing dynamic response CSVs:")
    for job in jobs:
        print(f"  {job['strategy']} run {job['run_index']}: {job['path']}")

    for job in jobs:
        ts, metrics, summary = process_one_run(job, args)
        ts_frames.append(ts)
        metric_frames.append(metrics)
        run_summaries.append(summary)

        print(
            f"[RUN] {summary['strategy']} run {summary['run_index']}: "
            f"rows={summary['raw_rows']} "
            f"torque_rejected={summary['torque_rejected_samples']} "
            f"({summary['torque_rejected_percent']:.2f}%) "
            f"status={summary['final_status']} fault={summary['fault_code']}"
        )

    raw_ts = pd.concat(ts_frames, ignore_index=True)
    metrics = pd.concat(metric_frames, ignore_index=True)
    run_summary = pd.DataFrame(run_summaries)
    agg_ts = aggregate_timeseries(raw_ts)
    summary = summarize_metrics(metrics)

    outdir = Path(args.outdir)

    plot_velocity(agg_ts, raw_ts, POSITIVE, outdir, args)
    plot_velocity(agg_ts, raw_ts, NEGATIVE, outdir, args)
    plot_torque(agg_ts, raw_ts, POSITIVE, outdir, args)
    plot_torque(agg_ts, raw_ts, NEGATIVE, outdir, args)

    plot_metric(summary, POSITIVE, "rise_time_s", "Rise Time Comparison", "Rise Time (s)", outdir, "dynamic_rise_time_comparison")
    plot_metric(summary, NEGATIVE, "fall_time_s", "Fall Time Comparison", "Fall Time (s)", outdir, "dynamic_fall_time_comparison")

    plot_metric(summary, POSITIVE, "overshoot_percent", "Positive Step Overshoot Comparison", "Overshoot (%)", outdir, "dynamic_positive_overshoot_comparison")
    plot_metric(summary, NEGATIVE, "overshoot_percent", "Negative Step Overshoot Comparison", "Overshoot (%)", outdir, "dynamic_negative_overshoot_comparison")

    plot_metric(summary, POSITIVE, "undershoot_percent", "Positive Step Undershoot Comparison", "Undershoot (%)", outdir, "dynamic_positive_undershoot_comparison")
    plot_metric(summary, NEGATIVE, "undershoot_percent", "Negative Step Undershoot Comparison", "Undershoot (%)", outdir, "dynamic_negative_undershoot_comparison")

    plot_metric(summary, POSITIVE, "settling_time_plot_s", "Positive Step Settling Time Comparison", "Settling Time (s)", outdir, "dynamic_positive_settling_time_comparison", dns=True)
    plot_metric(summary, NEGATIVE, "settling_time_plot_s", "Negative Step Settling Time Comparison", "Settling Time (s)", outdir, "dynamic_negative_settling_time_comparison", dns=True)

    write_outputs(outdir, raw_ts, agg_ts, metrics, summary, run_summary, settings_table(args))

    print("[DONE] Dynamic response processing, aggregation, and plotting complete.")


if __name__ == "__main__":
    main()