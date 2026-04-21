from __future__ import annotations

import matplotlib.pyplot as plt

from _plot_common import add_comparison_line, load_comparison_series, setup_axes


def main(show: bool = True) -> None:
    comparison_series = load_comparison_series()

    figure, axes = setup_axes(
        "Rotation Matrix Comparison",
        ["R00", "R01", "R02", "R11", "R12", "R22"],
    )

    for axis in axes:
        axis.set_ylim(-1.0, 1.0)

    add_comparison_line(axes[0], comparison_series, "R00", "R00")
    add_comparison_line(axes[1], comparison_series, "R01", "R01")
    add_comparison_line(axes[2], comparison_series, "R02", "R02")
    add_comparison_line(axes[3], comparison_series, "R11", "R11")
    add_comparison_line(axes[4], comparison_series, "R12", "R12")
    add_comparison_line(axes[5], comparison_series, "R22", "R22")

    figure.tight_layout()
    if show:
        plt.show()


if __name__ == "__main__":
    main()
