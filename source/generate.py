"""
Signal generation module.
"""

import numpy as np


def generate_inputs_for_quadrocopter(
    input_type: str, grid_length: int
) -> tuple:
    """
    Generate input omegas for quadrocopter simulation;
    :param input_type: type of control input;
    :param grid_length: length of time grid;
    :return: tuple of omegas.
    """
    if input_type == "constant":
        w_1 = 600 * np.ones(grid_length)
        w_2 = 600 * np.ones(grid_length)
        w_3 = 600 * np.ones(grid_length)
        w_4 = 600 * np.ones(grid_length)

    elif input_type == "updown":
        w_1 = 600 * np.ones(grid_length)
        w_2 = 600 * np.ones(grid_length)
        w_3 = 600 * np.ones(grid_length)
        w_4 = 600 * np.ones(grid_length)

        w_1[grid_length // 5 :] = 440
        w_2[grid_length // 5 :] = 440
        w_3[grid_length // 5 :] = 440
        w_4[grid_length // 5 :] = 440

    elif input_type == "jitter":
        w_1 = 800 * np.ones(grid_length)
        w_2 = 800 * np.ones(grid_length)
        w_3 = 800 * np.ones(grid_length)
        w_4 = 800 * np.ones(grid_length)

        w_1[grid_length // 50 :] = 488
        w_2[grid_length // 50 :] = 488
        w_3[grid_length // 50 :] = 488
        w_4[grid_length // 50 :] = 488

    elif input_type == "diagonal":
        w_1 = 800 * np.ones(grid_length)
        w_2 = 800 * np.ones(grid_length)
        w_3 = 850 * np.ones(grid_length)
        w_4 = 850 * np.ones(grid_length)

    elif input_type == "forward":
        w_1 = 800 * np.ones(grid_length)
        w_2 = 800 * np.ones(grid_length)
        w_3 = 800 * np.ones(grid_length)
        w_4 = 850 * np.ones(grid_length)

    else:
        raise NotImplementedError

    return w_1, w_2, w_3, w_4


def generate_inputs_for_hexacopter(input_type: str, grid_length: int) -> tuple:
    """
    Generate input omegas for hexacopter simulation;
    :param input_type: type of control input;
    :param grid_length: length of time grid;
    :return: tuple of omegas.
    """
    if input_type == "constant":
        w_1 = 600 * np.ones(grid_length)
        w_2 = 600 * np.ones(grid_length)
        w_3 = 600 * np.ones(grid_length)
        w_4 = 600 * np.ones(grid_length)
        w_5 = 600 * np.ones(grid_length)
        w_6 = 600 * np.ones(grid_length)

    elif input_type == "updown":
        w_1 = 600 * np.ones(grid_length)
        w_2 = 600 * np.ones(grid_length)
        w_3 = 600 * np.ones(grid_length)
        w_4 = 600 * np.ones(grid_length)
        w_5 = 600 * np.ones(grid_length)
        w_6 = 600 * np.ones(grid_length)

        w_1[grid_length // 3 :] = 325
        w_2[grid_length // 3 :] = 325
        w_3[grid_length // 3 :] = 325
        w_4[grid_length // 3 :] = 325
        w_5[grid_length // 3 :] = 325
        w_6[grid_length // 3 :] = 325

    elif input_type == "jitter":
        w_1 = 800 * np.ones(grid_length)
        w_2 = 800 * np.ones(grid_length)
        w_3 = 800 * np.ones(grid_length)
        w_4 = 800 * np.ones(grid_length)
        w_5 = 800 * np.ones(grid_length)
        w_6 = 800 * np.ones(grid_length)

        w_1[grid_length // 50 :] = 488
        w_2[grid_length // 50 :] = 488
        w_3[grid_length // 50 :] = 488
        w_4[grid_length // 50 :] = 488
        w_5[grid_length // 50 :] = 488
        w_6[grid_length // 50 :] = 488

    elif input_type == "diagonal":
        w_1 = 800 * np.ones(grid_length)
        w_2 = 800 * np.ones(grid_length)
        w_3 = 850 * np.ones(grid_length)
        w_4 = 850 * np.ones(grid_length)
        w_5 = 850 * np.ones(grid_length)
        w_6 = 850 * np.ones(grid_length)

    elif input_type == "forward":
        w_1 = 800 * np.ones(grid_length)
        w_2 = 800 * np.ones(grid_length)
        w_3 = 800 * np.ones(grid_length)
        w_4 = 800 * np.ones(grid_length)
        w_5 = 850 * np.ones(grid_length)
        w_6 = 850 * np.ones(grid_length)

    else:
        raise NotImplementedError

    return w_1, w_2, w_3, w_4, w_5, w_6
