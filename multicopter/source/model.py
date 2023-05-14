"""
Multi-copter module.
"""

import warnings
import yaml

import numpy as np
from scipy.integrate import odeint
from scipy.signal import place_poles


warnings.simplefilter("ignore")


class MultiCopter:
    """
    Copter with n rotors.
    """

    # Globals
    g: float

    # Copter parameters
    num_rotors: int
    rotor_angels: np.array
    m: float
    l_c: float
    k: np.array
    b: np.array

    # Inertia moments
    j_x: float
    j_y: float
    j_z: float

    def __init__(self, config_path: str):
        """
        Multi-copter initial;
        :param config_path: path to YAML config.
        """

        # Load config
        with open(config_path, "r", encoding="UTF-8") as stream:
            try:
                data = yaml.safe_load(stream)
                self.g = data["G"]
                self.m = data["M"]
                self.l_c = data["L"]
                self.k = data["K"]
                self.b = data["B"]

                self.num_rotors = data["NUM_ROTORS"]
                self.rotor_angels = np.deg2rad(np.array(data["ROTOR_ANGELS"]))

                self.j_x = data["J_X"]
                self.j_y = data["J_Y"]
                self.j_z = data["J_Z"]

            except yaml.YAMLError as exc:
                print(exc)

        self.control_input = None
        self.control_inputs = []

        # Simulation variables
        self.optimal_phase = None

        # Stabilization variables
        self.a_state = None
        self.b_state = None
        self.k_state = None

    def __generate_dynamic_matrix(self) -> np.array:
        matrix = np.zeros((4, self.num_rotors))

        # Deltas
        deltas = np.ones(self.num_rotors)
        for i in range(len(deltas)):
            if i % 2 != 0:
                deltas[i] = -1

        for i in range(self.num_rotors):
            # Fill 1st row
            matrix[0][i] = self.l_c * self.k * np.cos(self.rotor_angels[i])

            # Fill 2nd row
            matrix[1][i] = self.b * deltas[i]

            # Fill 3rd row
            matrix[2][i] = -self.l_c * self.k * np.sin(self.rotor_angels[i])

            # Fill 4th row
            matrix[3][i] = self.k

        return matrix

    @staticmethod
    def __place(a: np.array, b: np.array, poles: np.array) -> np.array:
        """
        Generate feedback matrix with desired poles.
        :param a: state-space matrix A;
        :param b: state-space matrix B;
        :param poles: desired poles of closed-loop system;
        :return: feedback matrix K.
        """
        result = place_poles(a, b, poles, method="YT")
        k = result.gain_matrix
        return k

    def __update_inputs(self, phase: np.array) -> None:
        """
        Update input variables for stabilization task;
        :param phase: phase vector;
        :return: None
        """
        self.control_input = -self.k_state @ (phase - self.optimal_phase)
        self.control_input[3] += self.g
        self.control_inputs.append(self.control_input)

    def get_rotor_speeds(self) -> np.array:
        """
        Get actual rotor speeds from control inputs;
        :return: array of rotor speeds in each time point.
        """
        dynamic_matrix = self.__generate_dynamic_matrix()
        if self.num_rotors > 4:
            dynamic_matrix = dynamic_matrix[:, [0, 1, 2, 3]]

        rotor_speeds = []

        for control_input in self.control_inputs:
            control_input[0] *= self.j_x
            control_input[1] *= self.j_y
            control_input[2] *= self.j_z
            control_input[3] *= self.m
            control_input[3] += self.g

            speeds = np.linalg.solve(dynamic_matrix, control_input)
            if self.num_rotors > 4:
                dummy_speeds = np.zeros(self.num_rotors - 4)
                speeds = np.concatenate((speeds, dummy_speeds))
            rotor_speeds.append(speeds)

        return np.sqrt(np.array(rotor_speeds))

    def simulate(
        self,
        in_phase: np.array,
        optimal_phase: np.array,
        num_points: int,
        simulation_time: int,
        is_linear: bool = False,
    ) -> tuple:
        """
        Run simulation;
        :param is_linear: simulate linear model;
        :param in_phase: input phase vector;
        :param optimal_phase: optimal phase vector;
        :param num_points: number of discrete points for simulation;
        :param simulation_time: time of simulation in seconds;
        :return: 12 arrays of phase coordinates.
        """
        self.optimal_phase = optimal_phase

        # State space matrices for linearizing
        self.a_state = np.array(
            [
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
                [0, 0, 0, 0, -self.g, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, self.g, 0, 0, 0, 0, 0, 0, 0, 0],
            ]
        )

        self.b_state = np.array(
            [
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 0],
                [0, 0, 0, 0],
                [0, 0, 0, 0],
                [0, 0, 0, 0],
                [0, 0, 0, 0],
                [0, 0, 0, 0],
                [0, 0, 0, 0],
                [0, 0, 0, 1],
                [0, 0, 0, 0],
            ]
        )

        # Placing poles
        poles = np.array(
            [-1, -1, -2, -2, -3, -3, -4, -4, -5, -5, -6, -6]
        )  # poles

        self.k_state = self.__place(
            self.a_state, self.b_state, poles
        )  # feedback matrix

        # Run solver
        cords_x = np.zeros(num_points)
        cords_y = np.zeros(num_points)
        cords_z = np.zeros(num_points)

        cords_x[0] = in_phase[6]
        cords_y[0] = in_phase[7]
        cords_z[0] = in_phase[8]

        grid = np.linspace(0, simulation_time, num_points)

        if is_linear:
            target_function = self.__model_linear
        else:
            target_function = self.__model_nonlinear

        solution = odeint(
            target_function,
            in_phase,
            grid,
        )

        omega_x = solution[:, 0]
        omega_y = solution[:, 1]
        omega_z = solution[:, 2]

        phi = solution[:, 3]
        theta = solution[:, 4]
        psi = solution[:, 5]

        cords_x = solution[:, 6]
        cords_y = solution[:, 7]
        cords_z = solution[:, 8]

        v_x = solution[:, 9]
        v_y = solution[:, 10]
        v_z = solution[:, 1]

        return (
            omega_x,
            omega_y,
            omega_z,
            cords_x,
            cords_y,
            cords_z,
            phi,
            theta,
            psi,
            v_x,
            v_y,
            v_z,
        )

    def __model_linear(self, x: np.array, _: np.array) -> tuple:
        """
        Simulation step of a linear model;
        :param x: current phase vector;
        :param _: time grid;
        :return: tuple of derivatives.
        """
        self.__update_inputs(x)
        return x @ (self.a_state - self.b_state @ self.k_state)

    def __model_nonlinear(self, x: np.array, _: np.array) -> tuple:
        """
        Simulation step of a non-linear model;
        :param x: current phase vector;
        :param _: time grid;
        :return: tuple of derivatives.
        """
        self.__update_inputs(x)

        x1, x2, x3, x4, x5, x6, _, _, _, x10, x11, x12 = x

        x1_dot = (
            self.j_y - self.j_z
        ) / self.j_x * x2 * x3 + self.control_input[0]
        x2_dot = (
            self.j_z - self.j_x
        ) / self.j_y * x1 * x3 + self.control_input[1]
        x3_dot = (
            self.j_x - self.j_y
        ) / self.j_z * x1 * x2 + self.control_input[2]

        x4_dot = x1 - np.tan(x5) * (x2 * np.cos(x4) - x3 * np.sin(x4))
        x5_dot = x2 * np.sin(x4) + x3 * np.cos(x4)
        x6_dot = 1 / np.cos(x5) * (x2 * np.cos(x4) - x3 * np.sin(x4))

        x7_dot = x10
        x8_dot = x11
        x9_dot = x12

        x10_dot = self.control_input[3] * (
            np.sin(x4) * np.sin(x6) - np.cos(x4) * np.sin(x5) * np.cos(x6)
        )
        x11_dot = self.control_input[3] * np.cos(x5) * np.cos(x4) - self.g
        x12_dot = self.control_input[3] * (
            np.cos(x6) * np.sin(x4) + np.cos(x4) * np.sin(x5) * np.sin(x6)
        )

        return (
            x1_dot,
            x2_dot,
            x3_dot,
            x4_dot,
            x5_dot,
            x6_dot,
            x7_dot,
            x8_dot,
            x9_dot,
            x10_dot,
            x11_dot,
            x12_dot,
        )
