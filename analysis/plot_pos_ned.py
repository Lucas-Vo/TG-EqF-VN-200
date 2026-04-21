from __future__ import annotations

import matplotlib.pyplot as plt

from _plot_common import add_comparison_line, load_comparison_series, setup_axes


def main(show: bool = True) -> None:
    comparison_series = load_comparison_series()

    figure, axes = setup_axes(
        "Position NED Comparison",
        ["north [m]", "east [m]", "down [m]"],
    )

    add_comparison_line(axes[0], comparison_series, "pos_N", "north")
    add_comparison_line(axes[1], comparison_series, "pos_E", "east")
    add_comparison_line(axes[2], comparison_series, "pos_D", "down")

    figure.tight_layout()
    if show:
        plt.show()


if __name__ == "__main__":
    main()
