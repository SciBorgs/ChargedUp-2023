package org.sciborgs1155.robot;

public final class Ports {

  public static final class OI {
    public static final int OPERATOR = 0;
    public static final int DRIVER = 1;
    public static final int LEFT_STICK = 2;
    public static final int RIGHT_STICK = 3;
  }
//angy
// angery
// the faint taste of peach - we should get sponsored by arizona
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

  public static final class Arm {
    public static final int LEFT_ELBOW_MOTOR = 14;
    public static final int MIDDLE_ELBOW_MOTOR = 15;
    public static final int RIGHT_ELBOW_MOTOR = 16;

    public static final int[] ELBOW_ENCODER = {8, 9};

    public static final int WRIST_MOTOR = 8;
  }

  public static final class Elevator {
    public static final int LEFT_MOTOR = 26;
    public static final int RIGHT_MOTOR = 25;
    public static final int MIDDLE_MOTOR = 24;

    public static final int[] ENCODER = {6, 7};
  }

  public static final class Intake {
    public static final int WHEEL_MOTOR = 6;
  }

  public static final class Led {
    public static final int led1 = 9;
    // on the robot before putting it on chassis the port # was 7
    public static final int led2 = 8;
  }
}
