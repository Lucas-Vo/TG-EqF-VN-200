from __future__ import annotations

import matplotlib.pyplot as plt

from _plot_common import LOG_DIR, add_comparison_line, load_series, setup_axes


def main() -> None:
    vn_data = load_series(LOG_DIR / "VNEstimate.csv")
    tgeqf_data = load_series(LOG_DIR / "TGEqFEstimate.csv")
    measurements_data = load_series(LOG_DIR / "Measurements.csv")

    figure, axes = setup_axes(
        "Angle-Axis Comparison",
        ["axis x", "axis y", "axis z", "cos(alpha)", "sin(alpha)"],
    )

    add_comparison_line(axes[0], vn_data, tgeqf_data, measurements_data, "angle_axis_x", "axis x")
    add_comparison_line(axes[1], vn_data, tgeqf_data, measurements_data, "angle_axis_y", "axis y")
    add_comparison_line(axes[2], vn_data, tgeqf_data, measurements_data, "angle_axis_z", "axis z")
    add_comparison_line(axes[3], vn_data, tgeqf_data, measurements_data, "angle_axis_cos", "cos(alpha)")
    add_comparison_line(axes[4], vn_data, tgeqf_data, measurements_data, "angle_axis_sin", "sin(alpha)")

    figure.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
