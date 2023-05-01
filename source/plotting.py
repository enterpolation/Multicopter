"""
Plotting module.
"""


import matplotlib
import matplotlib.pyplot as plt
import numpy as np

font = {"size": 12}
matplotlib.rc("font", **font)


def plot_speeds(
    speeds: np.array,
    plot_name: str,
    save_plot: bool = False
) -> None:
    n_rotors = speeds.shape[1]

    plt.figure(figsize=(12, 6))
    plt.title(r"Rotor angle speeds")
    plt.xlabel("iteration")
    plt.ylabel("rad / s")

    for i in range(n_rotors):
        plt.plot(speeds[:, i], label=f"$\omega_{i}$")

    plt.grid()
    plt.legend()

    if save_plot:
        plt.savefig(plot_name + ".eps")

    plt.show()


def plot_phase(
    omegas: tuple,
    cords: tuple,
    angles: tuple,
    speeds: tuple,
    grid: np.array,
    plot_name: str,
    save_plot: bool = False,
) -> None:
    """
    Plotting simulation phase variables;
    :param omegas: angle speeds;
    :param cords: space coordinates;
    :param angles: euler angles;
    :param speeds: projection speeds;
    :param grid: time grid;
    :param plot_name: name of a saved plot;
    :param save_plot: if save plots;
    :return: None.
    """
    omegas_x, omegas_y, omegas_z = omegas
    x_cords, y_cords, z_cords = cords
    phi, theta, psi = angles
    v_x_speed, v_y_speed, v_z_speed = speeds

    plt.figure(figsize=(18, 10))
    # plt.suptitle("Simulation plots")

    plt.subplot(221)
    plt.title("Coordinates $X$, $Y$, $Z$")
    plt.scatter(grid[0], x_cords[0], marker="x")
    plt.scatter(grid[len(grid) - 1], x_cords[len(grid) - 1], marker="o")
    plt.plot(grid, x_cords, label="$X$")

    plt.scatter(grid[0], y_cords[0], marker="x")
    plt.scatter(grid[len(grid) - 1], y_cords[len(grid) - 1], marker="o")
    plt.plot(grid, y_cords, label="$Y$")

    plt.scatter(grid[0], z_cords[0], marker="x")
    plt.scatter(grid[len(grid) - 1], z_cords[len(grid) - 1], marker="o")
    plt.plot(grid, z_cords, label="$Z$")

    plt.grid()
    plt.legend()

    plt.subplot(222)
    plt.title("Projection speeds $V_z, V_y, V_z$")
    plt.scatter(grid[0], v_x_speed[0], marker="x")
    plt.scatter(grid[len(grid) - 1], v_x_speed[len(grid) - 1], marker="o")
    plt.plot(grid, v_x_speed, label="$V_x$")

    plt.scatter(grid[0], v_y_speed[0], marker="x")
    plt.scatter(grid[len(grid) - 1], v_y_speed[len(grid) - 1], marker="o")
    plt.plot(grid, v_y_speed, label="$V_y$")

    plt.scatter(grid[0], v_z_speed[0], marker="x")
    plt.scatter(grid[len(grid) - 1], v_z_speed[len(grid) - 1], marker="o")
    plt.plot(grid, v_z_speed, label="$V_z$")

    plt.grid()
    plt.legend()

    plt.subplot(223)
    plt.title(r"Euler angles $\varphi, \theta, \psi$")
    plt.scatter(grid[0], phi[0], marker="x")
    plt.scatter(grid[len(grid) - 1], phi[len(grid) - 1], marker="o")
    plt.plot(grid, phi, label=r"$\varphi$")

    plt.scatter(grid[0], theta[0], marker="x")
    plt.scatter(grid[len(grid) - 1], theta[len(grid) - 1], marker="o")
    plt.plot(grid, theta, label=r"$\theta$")

    plt.scatter(grid[0], psi[0], marker="x")
    plt.scatter(grid[len(grid) - 1], psi[len(grid) - 1], marker="o")
    plt.plot(grid, psi, label=r"$\psi$")

    plt.grid()
    plt.legend()

    plt.subplot(224)
    plt.title(r"Angle speeds $\omega_x, \omega_y, \omega_z$")
    plt.scatter(grid[0], omegas_x[0], marker="x")
    plt.scatter(grid[len(grid) - 1], omegas_x[len(grid) - 1], marker="o")
    plt.plot(grid, omegas_x, label=r"$\omega_x$")

    plt.scatter(grid[0], omegas_y[0], marker="x")
    plt.scatter(grid[len(grid) - 1], omegas_y[len(grid) - 1], marker="o")
    plt.plot(grid, omegas_y, label=r"$\omega_y$")

    plt.scatter(grid[0], omegas_z[0], marker="x")
    plt.scatter(grid[len(grid) - 1], omegas_z[len(grid) - 1], marker="o")
    plt.plot(grid, omegas_z, label=r"$\omega_z$")
    plt.xlabel("Time (s)")

    plt.grid()
    plt.legend()

    if save_plot:
        plt.savefig(plot_name + ".eps")

    plt.show()


