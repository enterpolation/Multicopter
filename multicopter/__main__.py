"""
Main module.
"""

import argparse
import numpy as np
from multicopter.source.model import MultiCopter
from multicopter.source.plotting import plot_phase, plot_speeds


def main(args) -> None:
    time_grid = np.linspace(0, args.simulation_time, args.number_of_points)
    model = MultiCopter(config_path=args.path_to_config)

    delta = np.ones(12)
    phase = np.zeros(12)
    phase[6] = 10
    phase[7] = 10
    phase[8] = 10

    (
        omega_x,
        omega_y,
        omega_z,
        x,
        y,
        z,
        phi,
        theta,
        psi,
        v_x,
        v_y,
        v_z,
    ) = model.simulate(
        phase + delta,
        phase,
        args.number_of_points,
        args.simulation_time,
        is_linear=False,
    )

    rotor_speeds = model.get_rotor_speeds()

    if args.plot_phase:
        plot_phase(
            (omega_x, omega_y, omega_z),
            (x, y, z),
            (phi, theta, psi),
            (v_x, v_y, v_z),
            time_grid,
            plot_name="./img/multicopter_linear",
            save_plot=True,
        )

    if args.plot_speeds:
        plot_speeds(
            rotor_speeds,
            plot_name="./img/octocopter_angle_speeds",
            save_plot=True,
        )


CONFIG_PATH: str = "config/octocopter.yaml"
SIMULATION_TIME: int = 20
NUM_POINTS: int = 100
TIME_GRID = np.linspace(0, SIMULATION_TIME, NUM_POINTS)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Multicopter simulation.")
    parser.add_argument(
        "--path_to_config", type=str, help="Path to multicopter configuration.", required=True
    )
    parser.add_argument("--simulation_time", type=int, help="Time in seconds.", required=True)
    parser.add_argument(
        "--number_of_points", type=int, help="Number of time points.", required=True
    )
    parser.add_argument("--plot_phase", action="store_true")
    parser.add_argument("--plot_speeds", action="store_true")
    main(parser.parse_args())
