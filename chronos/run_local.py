# Copyright (c) 2023 FRC 6328
# http://github.com/Mechanical-Advantage

import json
from math import pi

from plotter import plot
from solver import Solver

if __name__ == "__main__":
    config = json.loads(open("src/main/deploy/arm_config.json", "r").read())
    solver = Solver(config)

    request = {
        "initial": [pi - (pi / 2.5), pi / 2],
        "final": [pi / 2.5, 3 * pi / 2],
        "constraintOverrides": [],
    }
    result = solver.solve(request)

    if result != None:
        print("DT =", result[0] / (len(result[1]) - 1))
        plot(result, config)