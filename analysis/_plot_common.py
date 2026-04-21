from __future__ import annotations

import csv
from pathlib import Path

import matplotlib.pyplot as plt


ROOT_DIR = Path(__file__).resolve().parents[1]
LOG_DIR = ROOT_DIR / "log"
VN_COLOR = "#d62728"
TGEQF_COLOR = "#6a5acd"
MEASUREMENTS_COLOR = "#32cd32"
IEEE_SERIF_FONTS = [
    "Times New Roman",
    "Times",
    "Nimbus Roman",
    "TeX Gyre Termes",
    "DejaVu Serif",
]


def load_series(path: Path) -> dict[str, list[float]]:
    if not path.exists():
        raise FileNotFoundError(f"Missing log file: {path}")

    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"No CSV header found in {path}")

        data = {field: [] for field in reader.fieldnames}
        for row in reader:
            for field in reader.fieldnames:
                data[field].append(float(row[field]))
        return data


def apply_ieee_style() -> None:
    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.serif": IEEE_SERIF_FONTS,
            "mathtext.fontset": "stix",
            "axes.titlesize": 13,
            "axes.titleweight": "bold",
            "axes.labelsize": 12,
            "xtick.labelsize": 11,
            "ytick.labelsize": 11,
            "legend.fontsize": 11,
            "legend.frameon": True,
            "legend.fancybox": False,
            "legend.framealpha": 1.0,
            "figure.titlesize": 14,
        }
    )


def setup_axes(
    title: str | None,
    ylabels: list[str],
    subplot_titles: list[str] | None = None,
) -> tuple[plt.Figure, list[plt.Axes]]:
    apply_ieee_style()
    figure, _ = plt.subplots(len(ylabels), 1, sharex=True, figsize=(12, 8))
    axes_list = list(figure.axes)

    if title:
        figure.suptitle(title, fontweight="bold")

    for axis, ylabel in zip(axes_list, ylabels):
        if ylabel:
            axis.set_ylabel(ylabel)
        axis.grid(True, alpha=0.3)

    if subplot_titles is not None:
        for axis, subplot_title in zip(axes_list, subplot_titles):
            axis.set_title(subplot_title, fontweight="bold", pad=8)

    axes_list[-1].set_xlabel("timestamp [s]")
    return figure, axes_list


def add_comparison_line(
    axis: plt.Axes,
    vn_data: dict[str, list[float]],
    tgeqf_data: dict[str, list[float]],
    measurements_data: dict[str, list[float]],
    column: str,
    label: str,
    show_legend: bool = True,
) -> None:
    axis.plot(vn_data["timestamp"], vn_data[column], color=VN_COLOR, label=f"VN200 {label}")
    axis.plot(
        tgeqf_data["timestamp"],
        tgeqf_data[column],
        color=TGEQF_COLOR,
        label=f"TGEqF {label}",
    )
    axis.plot(
        measurements_data["timestamp"],
        measurements_data[column],
        color=MEASUREMENTS_COLOR,
        label=f"Measurements {label}",
    )
    if show_legend:
        axis.legend(loc="best")


def add_shared_legend(figure: plt.Figure, axes: list[plt.Axes]) -> None:
    legend_entries = {}
    for axis in axes:
        handles, labels = axis.get_legend_handles_labels()
        for handle, label in zip(handles, labels):
            if label and label != "_nolegend_" and label not in legend_entries:
                legend_entries[label] = handle

    if legend_entries:
        figure.legend(
            legend_entries.values(),
            legend_entries.keys(),
            loc="upper center",
            bbox_to_anchor=(0.5, 0.98),
            ncol=len(legend_entries),
        )
