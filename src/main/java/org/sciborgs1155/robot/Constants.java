package org.sciborgs1155.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import java.awt.Color;
import java.util.Map;
import org.sciborgs1155.lib.constants.MotorConfig;
import org.sciborgs1155.lib.constants.MotorConfig.NeutralBehavior;
import org.sciborgs1155.lib.constants.PIDConstants;

/**
 * Constants is a globally accessible class for storing immutable values. Every value should be
 * <code>public static final</code>.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 *
 * <p><b>Units</b>
 *
 * <ul>
 *   <li>length: meters
 *   <li>time: seconds
 *   <li>angle: radians
 * </ul>
 *
 * @see MotorConfig
 * @see PIDConstants
 * @see Constraints
 */
public final class Constants {

  public static enum RobotType {
    WHIPLASH_ROLLER,
    WHIPLASH_CLAW,
    CHASSIS;
  }

  public static final double PERIOD = 0.02; // roborio tickrate (s)
  public static final double SPARK_PERIOD = 0.001; // SparkMAX tickrate (s)
  public static final double DEADBAND = 0.1;
  public static final int THROUGHBORE_PPR = 2048;

  public static final RobotType ROBOT_TYPE = RobotType.WHIPLASH_ROLLER;

  public static final class Dimensions {
    public static final double BASE_OFFSET = -0.127;
    // Distance from the center of the robot to the center of the elevator

    public static final double BASE_HEIGHT = 0.5203698;
    // Distance from the ground to the lowest possible elbow position

    public static final Translation2d BASE = new Translation2d(BASE_OFFSET, BASE_HEIGHT);

    public static final double FOREARM_LENGTH = 0.927;
    // Distance from elbow pivot to to wrist pivot

    public static final double CLAW_LENGTH = 0.3048;
    public static final double CLAW_LENGTH_OLD = 0.488;
    // Distance (hypotenuse) from wrist joint to tip

    public static final double CARRIAGE_MASS = 6.80389;
    // Mass of the carriage alone

    public static final double FOREARM_MASS = 4.08233;
    // Mass of the forearm alone

    public static final double CLAW_MASS = 2.803201;
    public static final double CLAW_MASS_OLD = 3.85554;
    // Mass of the claw alone
  }

  public static final class Auto {
    public static final double CUBE_OUTTAKE_TIME = 0.5; // seconds
    public static final double CONE_OUTTAKE_TIME = 3; // seconds
    public static final double INITIAL_INTAKE_TIME = 0.3; // seconds
    public static final double MOVING_INTAKE_TIME = 4; // seconds

    public static final PIDConstants BALANCE = new PIDConstants(0.05, 0, 0);

    public static final double PITCH_TOLERANCE = 12.5; // 12.5; // deg
  }

  public static final class Field {
    public static final double FIELD_WIDTH_METERS = 8.02;
    public static final double FIELD_LENGTH_METERS = 16.54;
    public static final Map<Integer, Translation2d> SCORING_POINTS_CUBE =
        Map.ofEntries(
            Map.entry(1, new Translation2d(1.83, 4.42)),
            Map.entry(2, new Translation2d(1.83, 2.75)),
            Map.entry(3, new Translation2d(1.83, 1.06)));
    public static final Map<Integer, Translation2d> SCORING_POINTS_CONE =
        Map.ofEntries(
            Map.entry(1, new Translation2d(1.83, 5.00)),
            Map.entry(2, new Translation2d(1.83, 3.85)),
            Map.entry(3, new Translation2d(1.83, 3.34)),
            Map.entry(4, new Translation2d(1.83, 2.17)),
            Map.entry(5, new Translation2d(1.83, 1.63)),
            Map.entry(6, new Translation2d(1.83, 0.51)));
  }

  public static final class LED {
    public static final int buffer1Length = 60;

    public static final int buffer2Length = 60;

    // RGB COLORS
    public static Color lightPurple = new Color(147, 112, 219);
    public static Color yellow = new Color(237, 237, 12);
    public static Color blue = new Color(0, 0, 228);
    public static Color rainbow1stPixel = new Color(255, 0, 0);
  }
}
