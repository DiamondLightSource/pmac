from enum import IntEnum

import matplotlib.pyplot as plt
import numpy as np

use_old_velocities = False


class VelMode(IntEnum):
    AvgPrevNext = 0
    RealPrevCurrent = 1
    AvgPrevCurrent = 2
    Zero = 3


# user programs
NO_PROGRAM = 0  # Do nothing
LIVE_PROGRAM = 1  # GPIO123 = 1, 0, 0
DEAD_PROGRAM = 2  # GPIO123 = 0, 1, 0
MID_PROGRAM = 4  # GPIO123 = 0, 0, 1
ZERO_PROGRAM = 8  # GPIO123 = 0, 0, 0


def velocity_prev_next(previous_pos, current_pos, next_pos, current_time, next_time):
    # proportional P->N velocity calculation (actually the same as before!)
    # speed1 = (current_pos - previous_pos) / current_time
    # speed2 = (next_pos - current_pos) / next_time
    # return (speed1 + speed2) / 2
    return (next_pos - previous_pos) / (current_time + next_time)


def avg_velocity_prev_current(prev_pos, current_pos, current_time):
    # redundant - now always use prev_current
    return (current_pos - prev_pos) / current_time


def velocity_prev_current(previous_pos, previous_velocity, current_pos, current_time):
    result, dt2 = 0, 0
    if use_old_velocities:
        result = old_velocity_prev_current(previous_pos, current_pos, current_time)
    elif current_time != 0:
        dt2 = 2.0 * (current_pos - previous_pos) / current_time
        result = dt2 - previous_velocity
    return result


def old_velocity_prev_current(previous_pos, current_pos, current_time):
    result = (current_pos - previous_pos) / current_time
    return result


def plot_pos_time(title, gather_x, gather_y, x, y, times):
    fig1 = plt.figure(figsize=(8, 6), dpi=200)

    plt.title(title)

    total_time = 0
    point_times = []
    for t in times:
        total_time += t / 1000000  # convert us to s
        point_times.append(total_time)

    plt.plot(point_times, y, marker="+", color="g", markersize=4)
    plt.plot(point_times, x, marker="+", color="r", markersize=4)
    # create an xgathaxis so that all points arrays are all same width
    gather_times = np.arange(0, total_time, total_time / len(gather_x))
    plt.plot(gather_times, gather_x, color="lightsalmon", linewidth=1)
    plt.plot(gather_times, gather_y, color="lightgreen", linewidth=1)

    plt.show()


def plot_velocities(
    np_arrays, title="Plot", step_time=0.15, overlay=None, x_scale=None, y_scale=None
):
    """ plots a 2d graph of a 2 axis trajectory, also does the velocity
    calculations and plots the velocity vector at each point.
    """
    xs, ys, ts, modes, user = np_arrays
    fig1 = plt.figure(figsize=(8, 6), dpi=200)
    axes = plt.gca()
    if x_scale:
        axes.set_xlim(x_scale)
    if y_scale:
        axes.set_ylim(y_scale)
    plt.title(title)

    # a multiply in ms for the velocity vector
    ms = step_time / 2 * 1000000

    # plot some velocity vectors
    vxs = np.zeros(len(xs) + 1)
    vys = np.zeros(len(xs) + 1)
    velocity_colors = ["#ff000030", "#aa605530", "#5500AA30", "#0060ff30"]

    for i in range(1, len(xs) - 1):
        if modes[i] == VelMode.AvgPrevNext:
            vxs[i] = velocity_prev_next(xs[i - 1], xs[i], xs[i + 1], ts[i], ts[i + 1])
            vys[i] = velocity_prev_next(ys[i - 1], ys[i], ys[i + 1], ts[i], ts[i + 1])
        elif modes[i] == VelMode.RealPrevCurrent:
            vxs[i] = velocity_prev_current(xs[i - 1], vxs[i - 1], xs[i], ts[i])
            vys[i] = velocity_prev_current(ys[i - 1], vys[i - 1], ys[i], ts[i])
        elif modes[i] == VelMode.AvgPrevCurrent:
            vxs[i] = avg_velocity_prev_current(xs[i - 1], xs[i], ts[i])
            vys[i] = avg_velocity_prev_current(ys[i - 1], ys[i], ts[i])

        # plot a line to represent the velocity vector
        s = "velocity vector {}: prev=({},{}) next=({},{}) " "vel=({},{}), time={}"
        print(
            s.format(
                i, xs[i - 1], ys[i - 1], xs[i], ys[i], vxs[i] * ms, vys[i] * ms, ts[i]
            )
        )

        ms = ts[i + 1]
        plt.plot(
            [xs[i], xs[i] + vxs[i] * ms],
            [ys[i], ys[i] + vys[i] * ms],
            color=velocity_colors[i % len(velocity_colors)],
        )

    # plot the start and end positions
    plt.plot([xs[0]], [ys[0]], "go", markersize=8)
    plt.plot([xs[-1]], [ys[-1]], "ro", markersize=8)

    for i in range(len(user)):
        if user[i] == MID_PROGRAM:
            # plot the data points with a black plus
            plt.plot(xs[i], ys[i], marker="+", color="k", markersize=8)
            # plot the bounds points with a dot
        elif user[i] == LIVE_PROGRAM:
            # plot over start of data with green dot
            plt.plot(xs[i], ys[i], marker=".", color="g", markersize=6)
        elif user[i] == DEAD_PROGRAM:
            # plot over end of data with red star
            plt.plot(xs[i], ys[i], marker="*", color="r", markersize=6)
        elif user[i] == ZERO_PROGRAM:
            # plot over GPIO 0 with yellow dot
            plt.plot(xs[i], ys[i], marker=".", color="y", markersize=6)
        elif user[i] == NO_PROGRAM:
            # plot turnaround points with a black dot
            plt.plot(xs[i], ys[i], linestyle="", marker=".", color="k", markersize=6)

    if overlay:
        plt.plot(
            overlay[0],
            overlay[1],
            linestyle="-",
            linewidth=0.01,
            marker="*",
            markersize=2,
            color="#8888ff",
        )

    plt.show()
