"""
Main module.
"""

import numpy as np

from source.model import MultiCopter
from source.plotting import plot_phase, plot_speeds


CONFIG_PATH: str = "./config/quadrocopter.yaml"

SIMULATION_TIME: int = 20
NUM_POINTS: int = 100

TIME_GRID = np.linspace(0, SIMULATION_TIME, NUM_POINTS)

if __name__ == "__main__":
    model = MultiCopter(config_path=CONFIG_PATH)

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
        NUM_POINTS,
        SIMULATION_TIME,
        is_linear=False
    )

    print(model.get_rotor_speeds())
    plot_speeds(model.get_rotor_speeds(), plot_name="angle_speeds", save_plot=True)
    plot_phase(
        (omega_x, omega_y, omega_z),
        (x, y, z),
        (phi, theta, psi),
        (v_x, v_y, v_z),
        TIME_GRID,
        plot_name="quadrocopter_linear",
        save_plot=True
    )
