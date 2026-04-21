from __future__ import annotations

import matplotlib.pyplot as plt

from _plot_common import add_comparison_line, load_comparison_series, setup_axes


def main(show: bool = True) -> None:
    comparison_series = load_comparison_series()

    figure, axes = setup_axes(
        "Acceleration XYZ Comparison",
        ["x [m/s^2]", "y [m/s^2]", "z [m/s^2]"],
    )

    add_comparison_line(axes[0], comparison_series, "accel_x", "x")
    add_comparison_line(axes[1], comparison_series, "accel_y", "y")
    add_comparison_line(axes[2], comparison_series, "accel_z", "z")

    figure.tight_layout()
    if show:
        plt.show()


if __name__ == "__main__":
    main()
