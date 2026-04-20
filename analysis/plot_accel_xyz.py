from __future__ import annotations

import matplotlib.pyplot as plt

from _plot_common import LOG_DIR, add_comparison_line, load_series, setup_axes


def main(show: bool = True) -> None:
    vn_data = load_series(LOG_DIR / "VNEstimate.csv")
    tgeqf_data = load_series(LOG_DIR / "TGEqFEstimate.csv")
    measurements_data = load_series(LOG_DIR / "Measurements.csv")

    figure, axes = setup_axes(
        "Acceleration XYZ Comparison",
        ["x [m/s^2]", "y [m/s^2]", "z [m/s^2]"],
    )

    add_comparison_line(axes[0], vn_data, tgeqf_data, measurements_data, "accel_x", "x")
    add_comparison_line(axes[1], vn_data, tgeqf_data, measurements_data, "accel_y", "y")
    add_comparison_line(axes[2], vn_data, tgeqf_data, measurements_data, "accel_z", "z")

    figure.tight_layout()
    if show:
        plt.show()


if __name__ == "__main__":
    main()
