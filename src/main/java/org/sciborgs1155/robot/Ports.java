package org.sciborgs1155.robot;

public final class Ports {

  public static final class OI {
    public static final int OPERATOR = 0;
    public static final int DRIVER = 1;
    public static final int LEFT_STICK = 2;
    public static final int RIGHT_STICK = 3;
  }

  public static final class Drive {
    public static final int FRONT_LEFT_DRIVE = 1;
    public static final int REAR_LEFT_DRIVE = 38;
    public static final int FRONT_RIGHT_DRIVE = 32;
    public static final int REAR_RIGHT_DRIVE = 5;

    public static final int FRONT_LEFT_TURNING = 31;
    public static final int REAR_LEFT_TURNING = 36;
    public static final int FRONT_RIGHT_TURNING = 3;
    public static final int REAR_RIGHT_TURNING = 40;

    public static final int PIGEON = 42;
  }

  public static final class Elbow {
    public static final int LEFT_MOTOR = 14;
    public static final int MIDDLE_MOTOR = 16;
    public static final int RIGHT_MOTOR = 15;

    public static final int[] ENCODER = {2, 3};
  }

  public static final class Wrist {
    public static final int MOTOR = 10;

    public static final int[] RELATIVE_ENCODER = {5, 6};
    public static final int ABS_ENCODER = 4;
  }

  public static final class Elevator {
    public static final int LEFT_MOTOR = 26;
    public static final int RIGHT_MOTOR = 24;
    public static final int MIDDLE_MOTOR = 25;

    public static final int[] ENCODER = {0, 1};
  }

  public static final class Intake {
    public static final int WHEEL_MOTOR = 7;
  }
}
