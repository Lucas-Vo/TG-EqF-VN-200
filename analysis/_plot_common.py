from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt


ROOT_DIR = Path(__file__).resolve().parents[1]
LOG_DIR = ROOT_DIR / "log"
VN_COLOR = "#d62728"
TGEQF_EQ_COLOR = "#6a5acd"
TGEQF_NEQ_COLOR = "#17becf"
MEASUREMENTS_COLOR = "#32cd32"


@dataclass(frozen=True)
class SeriesSpec:
    path: Path
    color: str
    label: str


SERIES_SPECS = (
    SeriesSpec(LOG_DIR / "VNEstimate.csv", VN_COLOR, "VN200"),
    SeriesSpec(
        LOG_DIR / "TGEqFEstimate_b_nu_eq_nu.csv",
        TGEQF_EQ_COLOR,
        r"TG-EqF $b_\nu = \nu$",
    ),
    SeriesSpec(
        LOG_DIR / "TGEqFEstimate_b_nu_neq_nu.csv",
        TGEQF_NEQ_COLOR,
        r"TG-EqF $b_\nu \neq \nu$",
    ),
    SeriesSpec(LOG_DIR / "Measurements.csv", MEASUREMENTS_COLOR, "Measurements"),
)


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


def load_comparison_series() -> list[tuple[SeriesSpec, dict[str, list[float]]]]:
    return [(series_spec, load_series(series_spec.path)) for series_spec in SERIES_SPECS]


def setup_axes(title: str, ylabels: list[str]) -> tuple[plt.Figure, list[plt.Axes]]:
    figure, _ = plt.subplots(len(ylabels), 1, sharex=True, figsize=(12, 8))
    axes_list = list(figure.axes)

    figure.suptitle(title)
    for axis, ylabel in zip(axes_list, ylabels):
        axis.set_ylabel(ylabel)
        axis.grid(True, alpha=0.3)

    axes_list[-1].set_xlabel("timestamp [s]")
    return figure, axes_list


def add_comparison_line(
    axis: plt.Axes,
    comparison_series: list[tuple[SeriesSpec, dict[str, list[float]]]],
    column: str,
    label: str,
) -> None:
    for series_spec, data in comparison_series:
        axis.plot(
            data["timestamp"],
            data[column],
            color=series_spec.color,
            label=f"{series_spec.label} {label}",
        )
    axis.legend(loc="best")
