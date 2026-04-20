from __future__ import annotations

import matplotlib.pyplot as plt

from _plot_common import LOG_DIR, add_comparison_line, load_series, setup_axes


def main(show: bool = True) -> None:
    vn_data = load_series(LOG_DIR / "VNEstimate.csv")
    tgeqf_data = load_series(LOG_DIR / "TGEqFEstimate.csv")
    measurements_data = load_series(LOG_DIR / "Measurements.csv")

    figure, axes = setup_axes(
        "Rotation Matrix Comparison",
        ["R00", "R01", "R02", "R11", "R12", "R22"],
    )

    for axis in axes:
        axis.set_ylim(-1.0, 1.0)

    add_comparison_line(axes[0], vn_data, tgeqf_data, measurements_data, "R00", "R00")
    add_comparison_line(axes[1], vn_data, tgeqf_data, measurements_data, "R01", "R01")
    add_comparison_line(axes[2], vn_data, tgeqf_data, measurements_data, "R02", "R02")
    add_comparison_line(axes[3], vn_data, tgeqf_data, measurements_data, "R11", "R11")
    add_comparison_line(axes[4], vn_data, tgeqf_data, measurements_data, "R12", "R12")
    add_comparison_line(axes[5], vn_data, tgeqf_data, measurements_data, "R22", "R22")

    figure.tight_layout()
    if show:
        plt.show()


if __name__ == "__main__":
    main()
