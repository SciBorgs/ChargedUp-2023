# Copyright (c) 2023 FRC 6328
# http://github.com/Mechanical-Advantage

import yaml
from math import pi

from plotter import plot
from solver import Solver

if __name__ == "__main__":
    with open("src/main/deploy/placement_config.yaml", "r") as f:
        config = yaml.safe_load(f)
    solver = Solver(config)

    request = {
        "initial": [0.2, 0, 7*pi/8],
        "final": [0.2, pi, pi],
        "constraintOverrides": [],
    }
    result = solver.solve(request)

    if result != None:
        print("DT =", result[0] / (len(result[1]) - 1))
        plot(result, config)