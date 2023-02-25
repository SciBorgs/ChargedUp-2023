# Copyright (c) 2023 FRC 6328
# http://github.com/Mechanical-Advantage

from dataclasses import dataclass

from casadi import *

class DCMotor:
    _nominalVoltageVolts: float = 0
    _stallTorqueNewtonMeters: float = 0
    _stallCurrentAmps: float = 0
    _freeCurrentAmps: float = 0
    _freeSpeedRadPerSec: float = 0
    _rOhms: float = 0
    _KvRadPerSecPerVolt: float = 0
    _KtNMPerAmp: float = 0

    def __init__(
        self,
        nominalVoltageVolts: float,
        stallTorqueNewtonMeters: float,
        stallCurrentAmps: float,
        freeCurrentAmps: float,
        freeSpeedRadPerSec: float,
        numMotors: int,
    ):
        self._nominalVoltageVolts = nominalVoltageVolts
        self._stallTorqueNewtonMeters = stallTorqueNewtonMeters * numMotors
        self._stallCurrentAmps = stallCurrentAmps * numMotors
        self._freeCurrentAmps = freeCurrentAmps * numMotors
        self._freeSpeedRadPerSec = freeSpeedRadPerSec

        self._rOhms = nominalVoltageVolts / self._stallCurrentAmps
        self._KvRadPerSecPerVolt = freeSpeedRadPerSec / (
            nominalVoltageVolts - self._rOhms * self._freeCurrentAmps
        )
        self._KtNMPerAmp = self._stallTorqueNewtonMeters / self._stallCurrentAmps

    def getVoltage(self, torqueNm, speedRadiansPerSec):
        return (
            1.0 / self._KvRadPerSecPerVolt * speedRadiansPerSec
            + 1.0 / self._KtNMPerAmp * self._rOhms * torqueNm
        )

    def withReduction(self, gearboxReduction: float):
        return DCMotor(
            self._nominalVoltageVolts,
            self._stallTorqueNewtonMeters * gearboxReduction,
            self._stallCurrentAmps,
            self._freeCurrentAmps,
            self._freeSpeedRadPerSec / gearboxReduction,
            1,
        )

    @classmethod
    def getNEO(self, numMotors):
        rpm = 5676
        return DCMotor(12, 2.6, 105, 1.8, rpm * math.pi / (60.0 / 2.0), numMotors)

    @classmethod
    def getNEO550(self, numMotors):
        rpm = 11000
        return DCMotor(12, 0.97, 100, 1.4, rpm * math.pi / (60.0 / 2.0), numMotors)

@dataclass
class JointConfig:
    mass: float
    length: float
    moi: float
    cgRadius: float
    motor: DCMotor

@dataclass
class LiftConfig:
    mass: float
    motor: DCMotor

class PlacementFeedforward:
    _g = 9.80665
    _elevator: LiftConfig
    _elbow: JointConfig
    _wrist: JointConfig

    def __init__(self, elevator: LiftConfig, elbow: JointConfig, wrist: JointConfig):
        self._elevator = elevator
        self._elbow = elbow
        self._wrist = wrist

    def calculate(self, position, velocity, acceleration):
        M = [[0, 0], [0, 0]]
        C = [[0, 0], [0, 0]]
        Tg = [0, 0]

        M[0][0] = (
            self._shoulder.mass * (self._shoulder.cgRadius**2.0)
            + self._elbow.mass
            * ((self._shoulder.length**2.0) + (self._elbow.cgRadius**2.0))
            + self._shoulder.moi
            + self._elbow.moi
            + 2
            * self._elbow.mass
            * self._shoulder.length
            * self._elbow.cgRadius
            * cos(position[1])
        )
        M[1][0] = (
            self._elbow.mass * (self._elbow.cgRadius**2)
            + self._elbow.moi
            + self._elbow.mass
            * self._shoulder.length
            * self._elbow.cgRadius
            * cos(position[1])
        )
        M[0][1] = (
            self._elbow.mass * (self._elbow.cgRadius**2)
            + self._elbow.moi
            + self._elbow.mass
            * self._shoulder.length
            * self._elbow.cgRadius
            * cos(position[1])
        )
        M[1][1] = self._elbow.mass * (self._elbow.cgRadius**2) + self._elbow.moi

        C[0][0] = (
            -self._elbow.mass
            * self._shoulder.length
            * self._elbow.cgRadius
            * sin(position[1])
            * velocity[1]
        )
        C[1][0] = (
            self._elbow.mass
            * self._shoulder.length
            * self._elbow.cgRadius
            * sin(position[1])
            * velocity[0]
        )
        C[0][1] = (
            -self._elbow.mass
            * self._shoulder.length
            * self._elbow.cgRadius
            * sin(position[1])
            * (velocity[0] + velocity[1])
        )

        Tg[0] = (
            self._shoulder.mass * self._shoulder.cgRadius
            + self._elbow.mass * self._shoulder.length
        ) * self._g * cos(
            position[0]
        ) + self._elbow.mass * self._elbow.cgRadius * self._g * cos(
            position[0] + position[1]
        )
        Tg[1] = (
            self._elbow.mass
            * self._elbow.cgRadius
            * self._g
            * cos(position[0] + position[1])
        )

        M_times_acceleration = (
            M[0][0] * acceleration[0] + M[0][1] * acceleration[1],
            M[1][0] * acceleration[0] + M[1][1] * acceleration[1],
        )
        C_times_velocity = (
            C[0][0] * velocity[0] + C[0][1] * velocity[1],
            C[1][0] * velocity[0] + C[1][1] * velocity[1],
        )
        torque = (
            M_times_acceleration[0] + C_times_velocity[0] + Tg[0],
            M_times_acceleration[1] + C_times_velocity[1] + Tg[1],
        )
        return (
            self._shoulder.motor.getVoltage(torque[0], velocity[0]),
            self._elbow.motor.getVoltage(torque[1], velocity[1]),
        )