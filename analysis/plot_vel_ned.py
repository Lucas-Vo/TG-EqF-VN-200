from __future__ import annotations

import matplotlib.pyplot as plt

from _plot_common import (
    LOG_DIR,
    add_comparison_line,
    add_shared_legend,
    load_series,
    setup_axes,
)


def main(show: bool = True) -> None:
    vn_data = load_series(LOG_DIR / "VNEstimate.csv")
    tgeqf_data = load_series(LOG_DIR / "TGEqFEstimate.csv")
    measurements_data = load_series(LOG_DIR / "Measurements.csv")

    figure, axes = setup_axes(
        None,
        ["", "", ""],
        subplot_titles=["North [m/s]", "East [m/s]", "Down [m/s]"],
    )

    add_comparison_line(axes[0], vn_data, tgeqf_data, measurements_data, "vel_N", "", show_legend=False)
    add_comparison_line(axes[1], vn_data, tgeqf_data, measurements_data, "vel_E", "", show_legend=False)
    add_comparison_line(axes[2], vn_data, tgeqf_data, measurements_data, "vel_D", "", show_legend=False)

    add_shared_legend(figure, axes)

    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.93))
    if show:
        plt.show()


if __name__ == "__main__":
    main()
