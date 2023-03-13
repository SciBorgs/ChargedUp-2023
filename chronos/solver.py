# Copyright (c) 2023 FRC 6328
# http://github.com/Mechanical-Advantage

import math

from casadi import *

from feedforward import PlacementFeedforward, JointConfig, LiftConfig, DCMotor

class Solver:
    def __init__(self, config: str, silence: bool = False):
        self._config = config

        # Create solver
        opti = Opti()
        self._opti = opti
        if silence:
            opti.solver("ipopt", {"print_time": 0}, {"print_level": 0, "sb": "yes"})
        else:
            opti.solver("ipopt")

        # Get constants from config
        n = config["solver"]["interiorPoints"]
        max_voltage_lift = config["solver"]["maxVoltageLift"]
        max_voltage_elbow = config["solver"]["maxVoltageElbow"]
        max_voltage_wrist = config["solver"]["maxVoltageWrist"]
        max_acceleration_lift = config["solver"]["maxAccelerationLift"]
        max_acceleration_elbow = config["solver"]["maxAccelerationElbow"]
        max_acceleration_wrist = config["solver"]["maxAccelerationWrist"]
        max_jerk = config["solver"]["maxJerk"]
        ff_model = PlacementFeedforward(
            LiftConfig(
                config["lift"]["mass"],
                self._get_motor(config["lift"]["motor"])
            ),
            JointConfig(
                config["elbow"]["mass"],
                config["elbow"]["length"],
                config["elbow"]["moi"],
                config["elbow"]["cgRadius"],
                self._get_motor(config["elbow"]["motor"]),
            ),
            JointConfig(
                config["wrist"]["mass"],
                config["wrist"]["length"],
                config["wrist"]["moi"],
                config["wrist"]["cgRadius"],
                self._get_motor(config["wrist"]["motor"]),
            ),
        )

        # Set up time variables
        self._total_time = opti.variable()
        dt = self._total_time / (n + 1)
        opti.subject_to(self._total_time > 0)
        opti.subject_to(self._total_time < 10)
        opti.minimize(self._total_time)

        # Create theta points
        positions = []
        self._positions = positions
        positions.append([opti.parameter(), opti.parameter(), opti.parameter()])
        for _ in range(n):
            height = opti.variable()
            theta_0 = opti.variable()
            theta_1 = opti.variable()
            opti.subject_to(
                opti.bounded(
                    config["lift"]["minHeight"],
                    height,
                    config["lift"]["maxHeight"]
                )
            )
            opti.subject_to(
                opti.bounded(
                    config["elbow"]["minAngle"],
                    theta_0,
                    config["elbow"]["maxAngle"],
                )
            )
            opti.subject_to(
                opti.bounded(
                    config["wrist"]["minRelAngle"], 
                    theta_1 - theta_0, 
                    config["wrist"]["maxRelAngle"]
                )
            )
            positions.append([height, theta_0, theta_1])
        positions.append([opti.parameter(), opti.parameter(), opti.parameter()])

        # Create constraint parameters (0 = disabled, 1 = enabled)
        self._constraint_parameters = {}
        for (constraint_key, constraint) in config["constraints"].items():
            self._constraint_parameters[constraint_key] = opti.parameter()

        # Apply point constraints
        for i in range(n + 2):
            # Get surrounding points
            last_theta_2 = positions[i if i <= 1 else i - 2]
            last_theta = positions[i if i == 0 else i - 1]
            current_theta = positions[i]
            next_theta = positions[i if i == n + 1 else i + 1]

            # Apply voltage constraints
            last_velocity_2 = (
                (last_theta[0] - last_theta_2[0]) / dt,
                (last_theta[1] - last_theta_2[1]) / dt,
                (last_theta[2] - last_theta_2[2]) / dt,
            )
            last_velocity = (
                (current_theta[0] - last_theta[0]) / dt,
                (current_theta[1] - last_theta[1]) / dt,
                (current_theta[2] - last_theta[2]) / dt,
            )
            next_velocity = (
                (next_theta[0] - current_theta[0]) / dt,
                (next_theta[1] - current_theta[1]) / dt,
                (next_theta[2] - current_theta[2]) / dt,
            )
            last_acceleration = (
                (last_velocity[0] - last_velocity_2[0]) / dt,
                (last_velocity[1] - last_velocity_2[1]) / dt,
                (last_velocity[2] - last_velocity_2[2]) / dt,
            )
            acceleration = (
                (next_velocity[0] - last_velocity[0]) / dt,
                (next_velocity[1] - last_velocity[1]) / dt,
                (next_velocity[2] - last_velocity[2]) / dt,
            )
            jerk = (
                (acceleration[0] - last_acceleration[0]) / (2 * dt),
                (acceleration[1] - last_acceleration[1]) / (2 * dt),
                (acceleration[2] - last_acceleration[2]) / (2 * dt),
            )
            opti.subject_to(
                opti.bounded(
                    -max_acceleration_lift, 
                    acceleration[0], 
                    max_acceleration_lift
                )
            )
            opti.subject_to(
                opti.bounded(
                    -max_acceleration_elbow,
                    acceleration[1],
                    max_acceleration_elbow,
                )
            )
            opti.subject_to(
                opti.bounded(
                    -max_acceleration_wrist, 
                    acceleration[2], 
                    max_acceleration_wrist
                )
            )
            voltage = ff_model.calculate(current_theta, last_velocity, acceleration)
            opti.subject_to(
                opti.bounded(-max_voltage_lift, voltage[0], max_voltage_lift)
            )
            opti.subject_to(
                opti.bounded(-max_voltage_elbow, voltage[1], max_voltage_elbow)
            )
            opti.subject_to(
                opti.bounded(-max_voltage_wrist, voltage[2], max_voltage_wrist)
            )
            opti.subject_to(opti.bounded(-max_jerk, jerk[0], max_jerk))
            opti.subject_to(opti.bounded(-max_jerk, jerk[1], max_jerk))
            opti.subject_to(opti.bounded(-max_jerk, jerk[2], max_jerk))

            # Apply position constraints
            if i != 0 and i != n + 1:
                points = {}
                points["origin"] = tuple(config["origin"])
                points["elbow"] = (
                    config["origin"][0],
                    config["origin"][1] + current_theta[0]
                )
                points["wrist"] = (
                    config["origin"][0]
                    + config["elbow"]["length"] * cos(current_theta[1]),
                    config["origin"][1]
                    + current_theta[0]
                    + config["elbow"]["length"] * sin(current_theta[1])
                )
                points["end"] = (
                    config["origin"][0]
                    + config["elbow"]["length"] * cos(current_theta[1])
                    + config["wrist"]["length"] * cos(current_theta[2]),
                    config["origin"][1]
                    + current_theta[0]
                    + config["elbow"]["length"] * sin(current_theta[1])
                    + config["wrist"]["length"] * sin(current_theta[2])
                )

                for (constraint_key, constraint) in config["constraints"].items():
                    enabled = self._constraint_parameters[constraint_key]
                    type = constraint["type"]
                    args = constraint["args"]
                    x, y = points[constraint["point"]]
                    if type == "minX":
                        opti.subject_to(x >= args[0] - ((1 - enabled) * 1000))

                    elif type == "maxX":
                        opti.subject_to(x <= args[0] + ((1 - enabled) * 1000))

                    elif type == "minY":
                        opti.subject_to(y >= args[0] - ((1 - enabled) * 1000))

                    elif type == "maxY":
                        opti.subject_to(y <= args[0] + ((1 - enabled) * 1000))

                    elif type == "circle":
                        center_x = args[0]
                        center_y = args[1]
                        radius = args[2]
                        opti.subject_to(
                            sqrt((x - center_x) ** 2 + (y - center_y) ** 2)
                            > radius * enabled
                        )

                    elif type == "rectangle":
                        x_0 = args[0]
                        y_0 = args[1]
                        x_1 = args[2]
                        y_1 = args[3]
                        opti.subject_to(
                            fmax(
                                fabs((2 * (x - ((x_0 + x_1) / 2)) / (x_1 - x_0))),
                                fabs((2 * (y - ((y_0 + y_1) / 2)) / (y_1 - y_0))),
                            )
                            > enabled
                        )

    def _get_motor(self, motor_config) -> DCMotor:
        if motor_config["type"] == "neo":
            return DCMotor.getNEO(motor_config["count"]).withReduction(
                motor_config["reduction"]
            )
        elif motor_config["type"] == "neo550":
            return DCMotor.getNEO550(motor_config["count"]).withReduction(
                motor_config["reduction"]
            )

    def solve(self, parameters):
        opti = self._opti
        positions = self._positions

        # Update position parameters
        opti.set_value(
            positions[0][0],
            self._clamp(
                parameters["initial"][0],
                self._config["lift"]["minHeight"],
                self._config["lift"]["maxHeight"],
            ),
        )
        opti.set_value(
            positions[0][1],
            self._clamp(
                parameters["initial"][1],
                self._config["elbow"]["minAngle"],
                self._config["elbow"]["maxAngle"],
            ),
        )
        opti.set_value(
            positions[0][2],
            self._clamp(
                parameters["initial"][1],
                self._config["elbow"]["minAngle"],
                self._config["elbow"]["maxAngle"],
            ) + self._clamp(
                parameters["initial"][2] - parameters["initial"][1],
                self._config["wrist"]["minRelAngle"],
                self._config["wrist"]["maxRelAngle"],
            ),
        )
        opti.set_value(
            positions[len(positions) - 1][0],
            self._clamp(
                parameters["final"][0],
                self._config["lift"]["minHeight"],
                self._config["lift"]["maxHeight"],
            ),
        )
        opti.set_value(
            positions[len(positions) - 1][1],
            self._clamp(
                parameters["final"][1],
                self._config["elbow"]["minAngle"],
                self._config["elbow"]["maxAngle"],
            ),
        )
        opti.set_value(
            positions[len(positions) - 1][2],
            self._clamp(
                parameters["final"][1],
                self._config["elbow"]["minAngle"],
                self._config["elbow"]["maxAngle"],
            ) + self._clamp(
                parameters["final"][2] - parameters["final"][1],
                self._config["wrist"]["minRelAngle"],
                self._config["wrist"]["maxRelAngle"],
            ),
        )

        # Set initial values
        opti.set_initial(self._total_time, 1)
        n = len(positions) - 2
        for i in range(1, n + 1):
            opti.set_initial(
                positions[i][0],
                (parameters["final"][0] - parameters["initial"][0]) * (i / (n + 2))
                + parameters["initial"][0],
            )
            opti.set_initial(
                positions[i][1],
                (parameters["final"][1] - parameters["initial"][1]) * (i / (n + 2))
                + parameters["initial"][1],
            )
            opti.set_initial(
                positions[i][2],
                (parameters["final"][2] - parameters["initial"][2]) * (i / (n + 2))
                + parameters["initial"][2],
            )

        # Set constraint parameters
        inital_theta = (opti.value(positions[0][0]), opti.value(positions[0][1]), opti.value(positions[0][2]))
        final_theta = (opti.value(positions[-1][0]), opti.value(positions[-1][1]), opti.value(positions[-1][2]))
        start_x = (
            self._config["origin"][0]
            + self._config["elbow"]["length"] * cos(inital_theta[1])
            + self._config["wrist"]["length"] * cos(inital_theta[2])
        )
        start_y = (
            self._config["origin"][1]
            + inital_theta[0]
            + self._config["elbow"]["length"] * sin(inital_theta[1])
            + self._config["wrist"]["length"] * sin(inital_theta[2])
        )
        final_x = (
            self._config["origin"][0]
            + self._config["elbow"]["length"] * cos(final_theta[1])
            + self._config["wrist"]["length"] * cos(final_theta[2])
        )
        final_y = (
            self._config["origin"][1]
            + final_theta[0]
            + self._config["elbow"]["length"] * sin(final_theta[1])
            + self._config["wrist"]["length"] * sin(final_theta[2])
        )
        for (
            constraint_key,
            constraint_parameter,
        ) in self._constraint_parameters.items():
            constraint = self._config["constraints"][constraint_key]
            type = constraint["type"]
            args = constraint["args"]

            # Check if start position violates the constraint
            any_violated = False
            for x, y, in [
                (start_x, start_y),
                (final_x, final_y),
            ]:
                violated = False
                if type == "minX":
                    violated = x < args[0]

                elif type == "maxX":
                    violated = x > args[0]

                elif type == "minY":
                    violated = y < args[0]

                elif type == "maxY":
                    violated = y > args[0]

                elif type == "circle":
                    center_x = args[0]
                    center_y = args[1]
                    radius = args[2]
                    violated = sqrt((x - center_x) ** 2 + (y - center_y) ** 2) <= radius

                elif type == "rectangle":
                    x_0 = args[0]
                    y_0 = args[1]
                    x_1 = args[2]
                    y_1 = args[3]
                    violated = x >= x_0 and x <= x_1 and y >= y_0 and y <= y_1

                if violated:
                    any_violated = True

            # Set enabled
            if any_violated or (constraint_key in parameters["constraintOverrides"]):
                opti.set_value(constraint_parameter, 0)
            else:
                opti.set_value(constraint_parameter, 1)

        # Solve
        opti.solve()

        # Get results
        result = (opti.value(self._total_time), [], [], [])
        for theta in positions:
            result[1].append(opti.value(theta[0]))
            result[2].append(opti.value(theta[1]))
            result[3].append(opti.value(theta[2]))
        return result

    def _clamp(self, value, min_value, max_value):
        return max(min_value, min(max_value, value))