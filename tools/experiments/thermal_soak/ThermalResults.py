#!/usr/bin/env python3
"""
Thermal comparison plotting script for OreSat reaction wheel thermal tests.

Produces thesis-ready V2 figures from ThermalSoakTest CSVs:

Figure X.X: Phase temperatures versus time
Figure X.X: MCU temperature versus time
Figure X.X: Temperature rise above ambient
Figure X.X: Final baseline-aligned steady-state temperatures

The raw CSV data is not modified. Display traces are baseline-aligned so that
sensor-to-sensor initial offset does not dominate the plots or reported final
statistics. Temperature rise plots are referenced to each channel's initial
measured baseline.

python ThermalSoakTest\ThermalResults.py --sine ThermalSoakTest\V2\thermal_soak_sine_raw.csv --trap ThermalSoakTest\V2\thermal_soak_trap_raw.csv --foc ThermalSoakTest\V2\thermal_soak_foc_raw.csv
"""

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


DEFAULT_OUTDIR = "Figures"
FINAL_WINDOW_S = 60.0

# Thermal plots only. Raw CSV data is preserved.
DEFAULT_MEDIAN_WINDOW_S = 3.0
DEFAULT_MEAN_WINDOW_S = 15.0
DEFAULT_BASELINE_WINDOW_S = 30.0

STRATEGY_FILES = {
    "Trapezoidal": "trap",
    "Sinusoidal": "sine",
    "FOC": "foc",
}

V2_PHASE_COLUMNS = {
    "Phase A": "v2_temp_phase_a_C",
    "Phase B": "v2_temp_phase_b_C",
    "Phase C": "v2_temp_phase_c_C",
}

V2_TEMP_COLUMNS = {
    **V2_PHASE_COLUMNS,
    "AUX": "v2_temp_mcu_C",
}

# Some V2 captures may not populate v2_temp_mcu_C yet. In that case, use the
# legacy packet field that V2 maps to the board/inverter auxiliary temperature.
# This keeps the MCU/board-temperature figure from going blank while preserving
# true v2_temp_mcu_C whenever it is available.
V2_MCU_FALLBACK_COLUMNS = [
    "v2_temp_mcu_C",
    "temp_mcu_C",
    "temp_inverter_C",
]


# =============================================================================
# DATA LOADING / VALIDATION
# =============================================================================

def read_csv_checked(path: Path) -> pd.DataFrame:
    if not path.exists():
        raise FileNotFoundError(f"Input CSV not found: {path}")

    df = pd.read_csv(path)
    if "host_time_s" not in df.columns:
        raise ValueError(f"{path} is missing required column: host_time_s")

    df = df.copy()
    df = df.sort_values("host_time_s").reset_index(drop=True)

    # Normalize every file to start at t=0 for clean thesis comparisons.
    t0 = pd.to_numeric(df["host_time_s"], errors="coerce").dropna()
    if not t0.empty:
        df["host_time_s"] = pd.to_numeric(df["host_time_s"], errors="coerce") - float(t0.iloc[0])

    df["time_min"] = df["host_time_s"] / 60.0
    return df


def load_datasets(args) -> dict:
    datasets = {}
    for label, attr in STRATEGY_FILES.items():
        value = getattr(args, attr)
        if value is None:
            continue
        datasets[label] = read_csv_checked(Path(value))

    if not datasets:
        raise ValueError("No input CSV files provided. Pass at least one of --trap, --sine, or --foc.")

    return datasets


def ensure_columns_exist(datasets: dict, temp_columns: dict):
    missing = []
    for strategy, df in datasets.items():
        for _label, column in temp_columns.items():
            if column not in df.columns:
                missing.append(f"{strategy}: {column}")

    if missing:
        raise ValueError("Missing required V2 thermal columns:\n" + "\n".join(missing))


def finite_series(df: pd.DataFrame, column: str) -> pd.Series:
    if column not in df.columns:
        return pd.Series(np.nan, index=df.index, dtype=float)
    return pd.to_numeric(df[column], errors="coerce")


def first_finite_column(df: pd.DataFrame, candidates: list[str]) -> str | None:
    for column in candidates:
        if column not in df.columns:
            continue

        y = pd.to_numeric(df[column], errors="coerce")
        if y.notna().any():
            return column

    return None


def finite_series_with_fallback(df: pd.DataFrame, candidates: list[str]) -> pd.Series:
    column = first_finite_column(df, candidates)
    if column is None:
        return pd.Series(np.nan, index=df.index, dtype=float)
    return finite_series(df, column)


def get_mcu_column_for_df(df: pd.DataFrame) -> str | None:
    return first_finite_column(df, V2_MCU_FALLBACK_COLUMNS)


def get_mcu_series_raw(df: pd.DataFrame) -> pd.Series:
    return finite_series_with_fallback(df, V2_MCU_FALLBACK_COLUMNS)


# =============================================================================
# SMOOTHING / BASELINE ALIGNMENT
# =============================================================================

def estimate_sample_period_s(df: pd.DataFrame) -> float:
    t = pd.to_numeric(df["host_time_s"], errors="coerce").to_numpy(dtype=float)
    t = t[np.isfinite(t)]
    if len(t) < 3:
        return 1.0

    dt = np.diff(t)
    dt = dt[np.isfinite(dt) & (dt > 0.0)]
    if len(dt) == 0:
        return 1.0

    return float(np.median(dt))


def window_seconds_to_samples(df: pd.DataFrame, window_s: float) -> int:
    if window_s <= 0.0:
        return 1

    dt_s = estimate_sample_period_s(df)
    samples = int(round(window_s / dt_s))
    return max(1, samples)


def smooth_temperature_series(
    df: pd.DataFrame,
    y: pd.Series,
    enable: bool,
    median_window_s: float,
    mean_window_s: float,
) -> pd.Series:
    y = pd.to_numeric(y, errors="coerce")

    if not enable:
        return y

    result = y.copy()

    median_n = window_seconds_to_samples(df, median_window_s)
    mean_n = window_seconds_to_samples(df, mean_window_s)

    if median_n > 1:
        result = result.rolling(
            window=median_n,
            center=True,
            min_periods=max(1, median_n // 3),
        ).median()

    if mean_n > 1:
        result = result.rolling(
            window=mean_n,
            center=True,
            min_periods=max(1, mean_n // 3),
        ).mean()

    return result.interpolate(limit_direction="both")


def get_temperature_series(
    df: pd.DataFrame,
    column: str,
    smooth: bool,
    median_window_s: float,
    mean_window_s: float,
) -> pd.Series:
    raw = finite_series(df, column)
    return smooth_temperature_series(
        df=df,
        y=raw,
        enable=smooth,
        median_window_s=median_window_s,
        mean_window_s=mean_window_s,
    )


def get_mcu_temperature_series(
    df: pd.DataFrame,
    smooth: bool,
    median_window_s: float,
    mean_window_s: float,
) -> pd.Series:
    raw = get_mcu_series_raw(df)
    return smooth_temperature_series(
        df=df,
        y=raw,
        enable=smooth,
        median_window_s=median_window_s,
        mean_window_s=mean_window_s,
    )


def initial_baseline_c(df: pd.DataFrame, y: pd.Series, baseline_window_s: float) -> float:
    n = window_seconds_to_samples(df, baseline_window_s)
    early = pd.to_numeric(y.iloc[:n], errors="coerce")
    return float(np.nanmedian(early.to_numpy(dtype=float)))


def aligned_to_common_start(
    df: pd.DataFrame,
    y: pd.Series,
    own_baseline_c: float,
    common_start_c: float,
) -> pd.Series:
    if not np.isfinite(own_baseline_c) or not np.isfinite(common_start_c):
        return y
    return y - own_baseline_c + common_start_c


def collect_series_and_baselines(
    datasets: dict,
    columns: dict,
    smooth: bool,
    median_window_s: float,
    mean_window_s: float,
    baseline_window_s: float,
):
    records = []

    for strategy, df in datasets.items():
        for label, column in columns.items():
            if label == "AUX":
                y = get_mcu_temperature_series(df, smooth, median_window_s, mean_window_s)
                resolved_column = get_mcu_column_for_df(df) or column
            else:
                y = get_temperature_series(df, column, smooth, median_window_s, mean_window_s)
                resolved_column = column

            baseline = initial_baseline_c(df, y, baseline_window_s)
            records.append({
                "strategy": strategy,
                "label": label,
                "column": resolved_column,
                "df": df,
                "y": y,
                "baseline": baseline,
            })

    baselines = [r["baseline"] for r in records if np.isfinite(r["baseline"])]
    common_start = float(np.nanmedian(baselines)) if baselines else np.nan
    return records, common_start


# =============================================================================
# PLOTTING HELPERS
# =============================================================================

def setup_axes(title: str, xlabel: str, ylabel: str):
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


def add_processing_note(ax, smooth: bool, median_window_s: float, mean_window_s: float, aligned: bool):
    notes = []
    if aligned:
        notes.append("Initial sensor offsets removed for display")
    if smooth:
        notes.append(f"Smoothed: {median_window_s:g}s median + {mean_window_s:g}s mean")

    if not notes:
        return

    ax.text(
        0.01,
        0.97,
        "\n".join(notes),
        transform=ax.transAxes,
        va="top",
        fontsize=8,
        bbox=dict(facecolor="white", alpha=0.75, edgecolor="none"),
    )


# =============================================================================
# REQUIRED THESIS FIGURES
# =============================================================================

def plot_phase_temperatures(
    datasets: dict,
    outdir: Path,
    smooth: bool,
    median_window_s: float,
    mean_window_s: float,
    baseline_window_s: float,
):
    fig, ax = setup_axes(
        "Phase Temperatures Versus Time",
        "Time (min)",
        "Temperature (°C, baseline-aligned)",
    )

    records, common_start = collect_series_and_baselines(
        datasets=datasets,
        columns=V2_PHASE_COLUMNS,
        smooth=smooth,
        median_window_s=median_window_s,
        mean_window_s=mean_window_s,
        baseline_window_s=baseline_window_s,
    )

    line_styles = {"Phase A": "-", "Phase B": "--", "Phase C": ":"}

    for r in records:
        y_plot = aligned_to_common_start(r["df"], r["y"], r["baseline"], common_start)
        ax.plot(
            r["df"]["time_min"],
            y_plot,
            linestyle=line_styles.get(r["label"], "-"),
            linewidth=1.6,
            label=f"{r['strategy']} {r['label']}",
        )

    add_processing_note(ax, smooth, median_window_s, mean_window_s, aligned=True)
    ax.legend(fontsize=8, ncols=2)
    save_figure(fig, outdir, "thermal_phase_temperatures_v2")


def plot_mcu_temperature(
    datasets: dict,
    outdir: Path,
    smooth: bool,
    median_window_s: float,
    mean_window_s: float,
    baseline_window_s: float,
):
    fig, ax = setup_axes(
        "AUX Temperature Versus Time",
        "Time (min)",
        "Temperature (°C, baseline-aligned)",
    )

    records, common_start = collect_series_and_baselines(
        datasets=datasets,
        columns={"AUX": "v2_temp_mcu_C"},
        smooth=smooth,
        median_window_s=median_window_s,
        mean_window_s=mean_window_s,
        baseline_window_s=baseline_window_s,
    )

    for r in records:
        y_plot = aligned_to_common_start(r["df"], r["y"], r["baseline"], common_start)
        ax.plot(r["df"]["time_min"], y_plot, linewidth=1.8, label=r["strategy"])

    add_processing_note(ax, smooth, median_window_s, mean_window_s, aligned=True)
    ax.legend()
    save_figure(fig, outdir, "thermal_mcu_temperature_v2")


def plot_temperature_rise(
    datasets: dict,
    outdir: Path,
    smooth: bool,
    median_window_s: float,
    mean_window_s: float,
    baseline_window_s: float,
):
    fig, ax = setup_axes(
        "Temperature Rise Above Ambient",
        "Time (min)",
        "Temperature Rise (°C)",
    )

    # Hottest V2 measured channel after subtracting each channel's own initial
    # measured baseline. This makes all traces start near 0 °C rise.
    for strategy, df in datasets.items():
        rise_matrix = []

        for label, column in V2_TEMP_COLUMNS.items():
            if label == "AUX":
                y = get_mcu_temperature_series(df, smooth, median_window_s, mean_window_s)
            else:
                y = get_temperature_series(df, column, smooth, median_window_s, mean_window_s)

            if y.notna().any():
                baseline = initial_baseline_c(df, y, baseline_window_s)
                rise_matrix.append((y - baseline).to_numpy(dtype=float))

        if not rise_matrix:
            continue

        hottest_rise = np.nanmax(np.vstack(rise_matrix), axis=0)
        ax.plot(df["time_min"], hottest_rise, linewidth=1.9, label=strategy)

    ax.axhline(0.0, linewidth=0.8)
    add_processing_note(ax, smooth, median_window_s, mean_window_s, aligned=False)
    ax.text(
        0.01,
        0.97 if not smooth else 0.86,
        f"Rise referenced to initial {baseline_window_s:g}s measured baseline",
        transform=ax.transAxes,
        va="top",
        fontsize=8,
        bbox=dict(facecolor="white", alpha=0.75, edgecolor="none"),
    )
    ax.legend()
    save_figure(fig, outdir, "thermal_temperature_rise_v2")


def final_window_mean_from_series(df: pd.DataFrame, y: pd.Series, final_window_s: float) -> float:
    """Return the mean of a processed series over the final time window."""
    t_end = float(df["host_time_s"].max())
    mask = df["host_time_s"] >= max(0.0, t_end - final_window_s)
    return float(pd.to_numeric(y[mask], errors="coerce").mean())


def make_final_temperature_table(
    datasets: dict,
    final_window_s: float,
    smooth: bool,
    median_window_s: float,
    mean_window_s: float,
    baseline_window_s: float,
) -> pd.DataFrame:
    """
    Compute final values from the same smoothed, baseline-aligned series used
    by the phase and AUX line plots.

    Phase channels share the phase-plot common start. AUX channels share the
    AUX-plot common start. This makes every value in the summary reproduce the
    corresponding line shown in its time-history figure.
    """
    phase_records, phase_common_start = collect_series_and_baselines(
        datasets=datasets,
        columns=V2_PHASE_COLUMNS,
        smooth=smooth,
        median_window_s=median_window_s,
        mean_window_s=mean_window_s,
        baseline_window_s=baseline_window_s,
    )
    aux_records, aux_common_start = collect_series_and_baselines(
        datasets=datasets,
        columns={"AUX": "v2_temp_mcu_C"},
        smooth=smooth,
        median_window_s=median_window_s,
        mean_window_s=mean_window_s,
        baseline_window_s=baseline_window_s,
    )

    rows = {strategy: {"strategy": strategy} for strategy in datasets}

    for r in phase_records:
        y_aligned = aligned_to_common_start(
            r["df"], r["y"], r["baseline"], phase_common_start
        )
        rows[r["strategy"]][r["label"]] = final_window_mean_from_series(
            r["df"], y_aligned, final_window_s
        )

    for r in aux_records:
        y_aligned = aligned_to_common_start(
            r["df"], r["y"], r["baseline"], aux_common_start
        )
        rows[r["strategy"]][r["label"]] = final_window_mean_from_series(
            r["df"], y_aligned, final_window_s
        )

    return pd.DataFrame([rows[strategy] for strategy in datasets])


def plot_final_steady_temperatures(summary: pd.DataFrame, outdir: Path):
    fig, ax = plt.subplots(figsize=(7.2, 4.2))

    labels = [
        c for c in summary.columns
        if c != "strategy" and pd.to_numeric(summary[c], errors="coerce").notna().any()
    ]
    x = np.arange(len(summary["strategy"]))
    width = 0.8 / max(1, len(labels))

    for i, label in enumerate(labels):
        offset = (i - (len(labels) - 1) / 2.0) * width
        ax.bar(x + offset, summary[label], width, label=label)

    ax.set_title("Final Baseline-Aligned Steady-State Temperatures")
    ax.set_xlabel("Commutation Strategy")
    ax.set_ylabel("Temperature (°C, baseline-aligned)")
    ax.set_xticks(x)
    ax.set_xticklabels(summary["strategy"])
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend(fontsize=8, ncols=2)

    save_figure(fig, outdir, "thermal_final_steady_temperatures_v2")


def write_summary(summary: pd.DataFrame, outdir: Path):
    outdir.mkdir(parents=True, exist_ok=True)
    path = outdir / "thermal_final_steady_temperatures_v2.csv"
    summary.to_csv(path, index=False)
    print(f"[TABLE] {path}")
    print("[STATS] Final steady-state temperatures (°C):")
    print(summary.to_string(index=False, float_format=lambda value: f"{value:.3f}"))


# =============================================================================
# MAIN
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description="Generate V2 OreSat thermal soak thesis figures.")

    parser.add_argument("--trap", help="Thermal CSV for trapezoidal commutation")
    parser.add_argument("--sine", help="Thermal CSV for sinusoidal commutation")
    parser.add_argument("--foc", help="Thermal CSV for FOC commutation")

    parser.add_argument("--outdir", default=DEFAULT_OUTDIR)
    parser.add_argument("--baseline-window-s", type=float, default=DEFAULT_BASELINE_WINDOW_S,
                        help="Initial measured baseline window used for display alignment and temperature rise.")
    parser.add_argument("--final-window-s", type=float, default=FINAL_WINDOW_S,
                        help="Averaging window at the end of the run for baseline-aligned steady-state temperatures.")

    parser.add_argument("--no-smoothing", action="store_true",
                        help="Disable thermal smoothing and plot raw temperature samples.")
    parser.add_argument("--median-window-s", type=float, default=DEFAULT_MEDIAN_WINDOW_S,
                        help="Rolling median window for display smoothing. Set to 0 to disable median stage.")
    parser.add_argument("--mean-window-s", type=float, default=DEFAULT_MEAN_WINDOW_S,
                        help="Rolling mean window for display smoothing. Set to 0 to disable mean stage.")

    args = parser.parse_args()

    outdir = Path(args.outdir)
    datasets = load_datasets(args)
    ensure_columns_exist(datasets, V2_PHASE_COLUMNS)

    for strategy, df in datasets.items():
        if get_mcu_column_for_df(df) is None:
            print(f"[WARN] {strategy}: no finite MCU/aux temperature source found; MCU plots will omit this strategy.")

    smooth = not args.no_smoothing

    plot_phase_temperatures(
        datasets=datasets,
        outdir=outdir,
        smooth=smooth,
        median_window_s=args.median_window_s,
        mean_window_s=args.mean_window_s,
        baseline_window_s=args.baseline_window_s,
    )

    plot_mcu_temperature(
        datasets=datasets,
        outdir=outdir,
        smooth=smooth,
        median_window_s=args.median_window_s,
        mean_window_s=args.mean_window_s,
        baseline_window_s=args.baseline_window_s,
    )

    plot_temperature_rise(
        datasets=datasets,
        outdir=outdir,
        smooth=smooth,
        median_window_s=args.median_window_s,
        mean_window_s=args.mean_window_s,
        baseline_window_s=args.baseline_window_s,
    )

    summary = make_final_temperature_table(
        datasets=datasets,
        final_window_s=args.final_window_s,
        smooth=smooth,
        median_window_s=args.median_window_s,
        mean_window_s=args.mean_window_s,
        baseline_window_s=args.baseline_window_s,
    )
    write_summary(summary, outdir)
    plot_final_steady_temperatures(summary, outdir)

    if smooth:
        print(f"[INFO] Thermal plot smoothing enabled: {args.median_window_s:g}s median + {args.mean_window_s:g}s mean")
    else:
        print("[INFO] Thermal plot smoothing disabled; raw temperature samples plotted.")
    print("[DONE] Thermal V2 thesis figures generated.")


if __name__ == "__main__":
    main()
