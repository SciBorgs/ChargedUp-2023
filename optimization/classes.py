# adapted from mechanical advantage

g = 9.802

@dataclass
class DCMotor:
    nominalVoltageVolts: float = 0
    stallTorqueNewtonMeters: float = 0
    stallCurrentAmps: float = 0
    freeCurrentAmps: float = 0
    freeSpeedRadPerSec: float = 0
    rOhms: float = 0
    KvRadPerSecPerVolt: float = 0
    KtNMPerAmp: float = 0

@dataclass
class Joint:
    mass: float
    length: float
    moi: float
    cgRadius: float
    motor: DCMotor

@dataclass
class Lift:
    mass: float
    length: float
    moi: float

@dataclass
class State:
    elevator: float
    shoulder: float
    wrist: float

class Arm:
    elevator: Lift
    shoulder: Joint
    wrist: Joint

    def __init__(self, elevator: Lift, shoulder: Joint, wrist: Joint):
        self.elevator = elevator
        self.shoulder = shoulder
        self.wrist = wrist

    def calculate(self, position: State, velocity: State, acceleration: State) -> State:
        return ()
