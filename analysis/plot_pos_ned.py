from __future__ import annotations

import matplotlib.pyplot as plt

from _plot_common import (
    LOG_DIR,
    MEASUREMENTS_COLOR,
    TGEQF_COLOR,
    VN_COLOR,
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
        subplot_titles=["North [m]", "East [m]", "Down [m]"],
    )

    series = [
        ("VN200", vn_data, VN_COLOR),
        ("TGEqF", tgeqf_data, TGEQF_COLOR),
        ("Measurements", measurements_data, MEASUREMENTS_COLOR),
    ]
    position_columns = ["pos_N", "pos_E", "pos_D"]

    for axis, column in zip(axes, position_columns):
        for label, data, color in series:
            axis.plot(data["timestamp"], data[column], color=color, label=label)

    add_shared_legend(figure, axes)

    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.93))
    if show:
        plt.show()


if __name__ == "__main__":
    main()
