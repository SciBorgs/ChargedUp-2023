# Copyright (c) 2023 FRC 6328
# http://github.com/Mechanical-Advantage
# Copyright (c) 2023 FRC 1155
# https://github.com/SciBorgs

import yaml
from math import pi

from plotter import plot
from solver import Solver

if __name__ == "__main__":
    with open("src/main/deploy/placement_config.yaml", "r") as f:
        config = yaml.safe_load(f)
    solver = Solver(config)

    request = {
        "initial": [0.455, -0.983, -0.09],
        "final": [0.2475, 3.072, 2.5],
        "constraintOverrides": [],
    }
    result = solver.solve(request)

    if result != None:
        print("DT =", result[0] / (len(result[1]) - 1))
        plot(result, config)