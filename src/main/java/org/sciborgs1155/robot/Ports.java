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
    public static final int LEFT_MOTOR = 8;
    public static final int RIGHT_MOTOR = 9;
    public static final int MIDDLE_MOTOR = 10;
    public static final int ELEVATOR_PORTS[] = {LEFT_MOTOR, RIGHT_MOTOR, MIDDLE_MOTOR};

    public static final int ELEVATOR_ENCODER_PORT = 2;
    public static final int BEAM_BREAK_PORTS[] = {5, 6};
  }

  public static final class ElbowPorts {
    public static final int LEFT_ELBOW_MOTOR = 11;
    public static final int RIGHT_ELBOW_MOTOR = 12;
    public static final int elbowPorts[] = {LEFT_ELBOW_MOTOR, RIGHT_ELBOW_MOTOR};
  }

  public static final class Sensors {
    public static final int PIGEON = 2;
  }

  public static final class ClawPorts {
    public static final int CLAW_WHEELS = 22;
    public static final int CLAW_WRIST = 23;
  }
}
