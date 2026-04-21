from __future__ import annotations

import matplotlib.pyplot as plt

from _plot_common import LOG_DIR, load_series, setup_axes


FROBENIUS_COLOR = "#111827"


def main(show: bool = True) -> None:
    frobenius_data = load_series(LOG_DIR / "fronebius.csv")

    figure, axes = setup_axes(
        "Covariance Difference Frobenius Norm",
        [r"$\|\Sigma_{reset} - \Sigma_{no\ reset}\|_F$"],
    )

    axes[0].plot(
        frobenius_data["timestamp"],
        frobenius_data["frobenius_norm"],
        color=FROBENIUS_COLOR,
        label=r"$\|\Sigma_{reset} - \Sigma_{no\ reset}\|_F$",
    )
    axes[0].legend(loc="best")

    figure.tight_layout()
    if show:
        plt.show()


if __name__ == "__main__":
    main()
