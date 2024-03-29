# Copyright (c) 2023 FRC 6328
# http://github.com/Mechanical-Advantage
# Copyright (c) 2023 FRC 1155
# https://github.com/SciBorgs

import math
import time

import matplotlib.animation
import matplotlib.pyplot as plt


def plot(result, config):
    # Get animation points
    dt = result[0] / (len(result[1]) - 1)
    animation_points = []
    current_time = 0
    for i in range(len(result[1])):
        animation_points.append((current_time, result[1][i], result[2][i], result[3][i]))
        current_time += dt

    # Set up plot
    fig, ax = plt.subplots()
    start_time = time.time()

    # Animate function
    def animate(i):
        current_time = (time.time() - start_time) % (result[0] + 2) - 1
        next_index = 0
        while (
            next_index < len(animation_points)
            and animation_points[next_index][0] < current_time
        ):
            next_index += 1
        if next_index >= len(animation_points):
            next_index = len(animation_points) - 1
        last_point = animation_points[next_index - 1]
        next_point = animation_points[next_index]

        t = (current_time - last_point[0]) / (next_point[0] - last_point[0])
        t = 1 if t > 1 else t
        t = 0 if t < 0 else t
        height = (next_point[1] - last_point[1]) * t + last_point[1]
        theta_1 = (next_point[2] - last_point[2]) * t + last_point[2]
        theta_2 = (next_point[3] - last_point[3]) * t + last_point[3]

        x = [
            config["origin"][0],
            config["origin"][0],
            config["origin"][0] + config["elbow"]["length"] * math.cos(theta_1),
            config["origin"][0]
            + config["elbow"]["length"] * math.cos(theta_1)
            + config["wrist"]["length"] * math.cos(theta_2),
        ]
        y = [
            config["origin"][1],
            config["origin"][1] + height,
            config["origin"][1] + height + config["elbow"]["length"] * math.sin(theta_1),
            config["origin"][1] + height
            + config["elbow"]["length"] * math.sin(theta_1)
            + config["wrist"]["length"] * math.sin(theta_2),
        ]
        ax.clear()
        ax.plot([-2, 2], [0, 0])
        ax.plot(x, y)

        ax.set_xlim([-2, 2])
        ax.set_ylim([-0.5, 2.6])

    # Show plot
    animation = matplotlib.animation.FuncAnimation(fig, animate, interval=20)
    plt.show()