from __future__ import annotations

import csv
from pathlib import Path

import matplotlib.pyplot as plt


ROOT_DIR = Path(__file__).resolve().parents[1]
LOG_DIR = ROOT_DIR / "log"
VN_COLOR = "#d62728"
TGEQF_COLOR = "#6a5acd"


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
    vn_data: dict[str, list[float]],
    tgeqf_data: dict[str, list[float]],
    column: str,
    label: str,
) -> None:
    axis.plot(vn_data["timestamp"], vn_data[column], color=VN_COLOR, label=f"VN200 {label}")
    axis.plot(
        tgeqf_data["timestamp"],
        tgeqf_data[column],
        color=TGEQF_COLOR,
        label=f"TGEqF {label}",
    )
    axis.legend(loc="best")
