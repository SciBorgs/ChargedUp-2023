package org.sciborgs1155.robot;

public final class Ports {

  public static final class OI {
    public static final int XBOX = 0;
    public static final int LEFT_STICK = 1;
    public static final int RIGHT_STICK = 2;
  }

  public static final class DrivePorts {
    public static final int FRONT_LEFT_DRIVE = 4;
    public static final int REAR_LEFT_DRIVE = 1;
    public static final int FRONT_RIGHT_DRIVE = 13;
    public static final int REAR_RIGHT_DRIVE = 5;

    public static final int FRONT_LEFT_TURNING = 7;
    public static final int REAR_LEFT_TURNING = 31;
    public static final int FRONT_RIGHT_TURNING = 32;
    public static final int REAR_RIGHT_TURNING = 3;

    // duty cycles for turning encoders
    public static final int FRONT_LEFT_DUTY_CYCLE = 0;
    public static final int FRONT_RIGHT_DUTY_CYCLE = 1;
    public static final int BACK_LEFT_DUTY_CYCLE = 2;
    public static final int BACK_RIGHT_DUTY_CYCLE = 3;
  }

  public static final class Sensors {
    public static final int PIGEON = 2;
  }
}
