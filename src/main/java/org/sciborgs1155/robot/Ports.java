package org.sciborgs1155.robot;

public final class Ports {

  public static final class OI {
    public static final int xbox = 0;
    public static final int leftStick = 1;
    public static final int rightStick = 2;
  }

  public static final class DrivePorts {
    public static final int frontLeftDriveMotorPort = 0;
    public static final int rearLeftDriveMotorPort = 2;
    public static final int frontRightDriveMotorPort = 4;
    public static final int rearRightDriveMotorPort = 6;

    public static final int frontLeftTurningMotorPort = 1;
    public static final int rearLeftTurningMotorPort = 3;
    public static final int frontRightTurningMotorPort = 5;
    public static final int rearRightTurningMotorPort = 7;
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
