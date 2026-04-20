from __future__ import annotations

import matplotlib.pyplot as plt

import plot_accel_xyz
import plot_angle_axis
import plot_pos_ned
import plot_vel_ned


def main() -> None:
    plot_angle_axis.main(show=False)
    plot_pos_ned.main(show=False)
    plot_vel_ned.main(show=False)
    plot_accel_xyz.main(show=False)
    plt.show()


if __name__ == "__main__":
    main()
