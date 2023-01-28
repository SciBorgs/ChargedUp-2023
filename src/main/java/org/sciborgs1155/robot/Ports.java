package org.sciborgs1155.robot;

public final class Ports {

  public static final class OI {
    public static final int XBOX = 0;
    public static final int LEFT_STICK = 1;
    public static final int RIGHT_STICK = 2;
  }

  public static final class DrivePorts {
    public static final int FRONT_LEFT_DRIVE = 38;
    public static final int REAR_LEFT_DRIVE = 1;
    public static final int FRONT_RIGHT_DRIVE = 5;
    public static final int REAR_RIGHT_DRIVE = 32;

    public static final int FRONT_LEFT_TURNING = 36;
    public static final int REAR_LEFT_TURNING = 31;
    public static final int FRONT_RIGHT_TURNING = 40;
    public static final int REAR_RIGHT_TURNING = 3;
  }

  public static final class ElevatorPorts {
    public static final int leftElevatorMotor = 8;
    public static final int rightElevatorMotor = 9;
    public static final int middleElevatorMotor = 10;
    public static final int elevatorPorts[] = {
      leftElevatorMotor, rightElevatorMotor, middleElevatorMotor
    };
  }

  public static final class ArmPorts {
    public static final int leftArmMotor = 11;
    public static final int rightArmMotor = 12;
    public static final int armPorts[] = {leftArmMotor, rightArmMotor};
  }

  public static final class Sensors {
    public static final int PIGEON = 2;
  }
}
