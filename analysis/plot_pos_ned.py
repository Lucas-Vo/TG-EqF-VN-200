from __future__ import annotations

import matplotlib.pyplot as plt

from _plot_common import LOG_DIR, add_comparison_line, load_series, setup_axes


def main() -> None:
    vn_data = load_series(LOG_DIR / "VNEstimate.csv")
    tgeqf_data = load_series(LOG_DIR / "TGEqFEstimate.csv")

    figure, axes = setup_axes(
        "Position NED Comparison",
        ["north [m]", "east [m]", "down [m]"],
    )

    add_comparison_line(axes[0], vn_data, tgeqf_data, "pos_N", "north")
    add_comparison_line(axes[1], vn_data, tgeqf_data, "pos_E", "east")
    add_comparison_line(axes[2], vn_data, tgeqf_data, "pos_D", "down")

    figure.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
