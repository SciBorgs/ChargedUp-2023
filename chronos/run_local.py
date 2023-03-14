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
        # "initial": [0.44, -1.110857, 0.1],
        "initial": [0.425006, 0.128855, -0.305],
        "final": [0.521769, 3.04, 2.74],
        "constraintOverrides": [],
    }
    result = solver.solve(request)

    if result != None:
        print("DT =", result[0] / (len(result[1]) - 1))
        plot(result, config)