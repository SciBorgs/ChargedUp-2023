package org.sciborgs1155.robot;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import java.awt.Color;
import java.util.Map;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.sciborgs1155.lib.constants.Conversion;
import org.sciborgs1155.lib.constants.Conversion.PulsesPerRev;
import org.sciborgs1155.lib.constants.MotorConfig;
import org.sciborgs1155.lib.constants.MotorConfig.NeutralBehavior;
import org.sciborgs1155.lib.constants.PIDConstants;
import org.sciborgs1155.lib.constants.SystemConstants;
import org.sciborgs1155.robot.Constants.Arm.Elbow;
import org.sciborgs1155.robot.util.placement.PlacementState;

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
 * @see Conversion
 * @see PIDConstants
 * @see Constraints
 */
public final class Constants {

  public static final double RATE = 0.02; // roborio tickrate (s)
  public static final double DEADBAND = 0.1;
  public static final int THROUGHBORE_CPR = 8192;

  public static final class Dimensions {
    public static final double BASE_OFFSET = -0.127;
    // Distance from the center of the robot to the center of the elevator

    public static final double BASE_HEIGHT = 0.5203698;
    // Distance from the ground to the lowest possible elbow position

    public static final double FOREARM_LENGTH = 0.927;
    // Distance from elbow pivot to to wrist pivot

    public static final double CLAW_LENGTH = 0.488;
    // Distance (hypotenuse) from wrist joint to tip

    public static final double CARRIAGE_MASS = 6.80389;
    // Mass of the carriage alone

    public static final double FOREARM_MASS = 4.08233;
    // Mass of the forearm alone

    public static final double CLAW_MASS = 3.85554;
    // Mass of the claw alone
  }

  public static final class Vision {
    // Camera names
    public static final String FRONT_CAM = "frontPhotonVision";
    public static final String BACK_CAM = "backPhotonVision";

    // Robot to camera translations
    public static final Translation3d FRONT_CAM_TRANSLATION =
        new Translation3d(0.165, -0.305, 0.356);
    public static final Rotation3d FRONT_CAM_ROTATION = new Rotation3d(1.64933614, 1.78, 0);
    public static final Transform3d ROBOT_TO_FRONT_CAM =
        new Transform3d(FRONT_CAM_TRANSLATION, FRONT_CAM_ROTATION);

    public static final Translation3d BACK_CAM_TRANSLATION =
        new Translation3d(); // Units.degreesToRadians(-180)
    public static final Rotation3d BACK_CAM_ROTATION =
        new Rotation3d(0, 0, Units.degreesToRadians(-180));
    public static final Transform3d ROBOT_TO_BACK_CAM =
        new Transform3d(BACK_CAM_TRANSLATION, BACK_CAM_ROTATION);

    public static final PoseStrategy PRIMARY_POSE_STRATEGY = PoseStrategy.LOWEST_AMBIGUITY;
    public static final PoseStrategy SECONDARY_POSE_STRATEGY = PoseStrategy.LOWEST_AMBIGUITY;

    public static final double LED_RANGE = 9000;
    public static final double CAM_FOV = 68.5;
    public static final double MIN_TARGET_AREA = 90;
    public static final int CAMERA_RES_WIDTH = 960;
    public static final int CAMERA_RES_HEIGHT = 544;
  }

  public static final class Arm {
    public static final class Wrist {
      public static final MotorConfig MOTOR =
          MotorConfig.base().withNeutralBehavior(NeutralBehavior.BRAKE).withInvert(true);

      public static final double GEARING = 53.125 / 1.0;
      // Gearing for motor : angle (radians)

      public static final Conversion CONVERSION =
          Conversion.base().withUnits(Conversion.Units.RADIANS);

      public static final PIDConstants PID = new PIDConstants(5.5, 0, 0.1); // p: 6.1297, d: 0.8453
      public static final SystemConstants FF =
          new SystemConstants(0.1542, 0.6, 0.91, 0.038046); // v =  0.87884

      public static final Constraints CONSTRAINTS = new Constraints(2.45, 2.9);

      public static final double MAX_ANGLE = Math.PI;
      public static final double MIN_ANGLE = -Math.PI;
    }

    public static final class Elbow {
      public static final MotorConfig MOTOR =
          MotorConfig.base().withNeutralBehavior(NeutralBehavior.BRAKE).withCurrentLimit(50);

      public static final double GEARING = 63.75 / 1.0;
      // Gearing for motor : angle (radians)

      public static final Conversion CONVERSION =
          Conversion.base()
              .multiplyGearing(12)
              .divideGearing(72)
              .withUnits(Conversion.Units.RADIANS)
              .withPulsesPerRev(PulsesPerRev.REV_INTEGRATED);

      public static final PIDConstants PID = new PIDConstants(12, 0, 1.1); // d = 2.18954
      public static final SystemConstants FF =
          new SystemConstants(0.020283, 0.71, 1.3174, 0.20891); // g = 0.63031;

      public static final Constraints CONSTRAINTS = new Constraints(2.9, 3.4);

      public static final double MAX_ANGLE = 3.0 * Math.PI / 2.0;
      public static final double MIN_ANGLE = -Math.PI / 2.0;

      public static final double ELBOW_OFFSET = -1.248660;

      public static final double FAILING_DEBOUNCE_TIME = 0.2;
    }
  }

  public static final class Elevator {
    public static final MotorConfig MOTOR =
        MotorConfig.base().withNeutralBehavior(NeutralBehavior.BRAKE).withCurrentLimit(35);

    public static final double GEARING = 30.0 / 1.0;
    // Gearing for motor : height (meters)

    public static final Conversion RELATIVE_CONVERSION =
        Conversion.base()
            .multiplyRadius(0.0181864)
            .withUnits(Conversion.Units.RADIANS)
            .withPulsesPerRev(PulsesPerRev.REV_INTEGRATED);
    // units field for sysid is 0.1143
    public static final Conversion ABSOLUTE_CONVERSION =
        RELATIVE_CONVERSION.withPulsesPerRev(PulsesPerRev.REV_INTEGRATED);

    public static final PIDConstants PID = new PIDConstants(50, 0, 1);
    public static final SystemConstants FF = new SystemConstants(0.4, 0.069335, 33.25, 1.5514);

    public static final double MIN_HEIGHT = 0;
    public static final double MAX_HEIGHT = 0.65;

    public static final int SAMPLE_SIZE_TAPS = 3;
    public static final int CURRENT_THRESHOLD = 40; // TODO this might be too high/unreasonable

    public static final Constraints CONSTRAINTS = new Constraints(3, 3.35);

    public static final double ZERO_OFFSET = 0.61842;

    public static final double FAILING_DEBOUNCE_TIME = 0.2;
  }

  public static final class Intake {
    public static final MotorConfig MOTOR =
        MotorConfig.base()
            .withNeutralBehavior(NeutralBehavior.BRAKE)
            .withCurrentLimit(20)
            .withInvert(true);

    public static final double DEFAULT_SPEED = 0.1;
    public static final double INTAKE_SPEED = 1;
    public static final double OUTTAKE_SPEED = -0.3;

    public static final double THRESHOLD = 0.5;
  }

  public static final class Drive {
    public enum SpeedMultiplier {
      NORMAL(1),
      SLOW(1.0 / 3.0);

      public final double multiplier;

      private SpeedMultiplier(double multiplier) {
        this.multiplier = multiplier;
      }
    }

    public static final double MAX_SPEED = 3.25; // m / s
    public static final double MAX_ANGULAR_SPEED = 1.5 * Math.PI; // rad / s
    public static final double MAX_ACCEL = 7.8; // m / s^2

    public static final double TRACK_WIDTH = 0.5715;
    // Distance between centers of right and left wheels on robot
    public static final double WHEEL_BASE = 0.5715;
    // Distance between front and back wheels on robot

    public static final Translation2d[] MODULE_OFFSET = {
      new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // front left
      new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // front right
      new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // rear left
      new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2) // rear right
    };

    // angular offsets of the modules, since we use absolute encoders
    // ignored (used as 0) in simulation because the simulated robot doesn't have offsets
    public static final double[] ANGULAR_OFFSETS = {
      -Math.PI / 2, // front left
      0, // front right
      Math.PI, // rear left
      Math.PI / 2 // rear right
    };

    public static final PIDConstants TRANSLATION = new PIDConstants(0.6, 0, 0);
    public static final PIDConstants ROTATION = new PIDConstants(0.4, 0, 0);

    public static final double MIN_PITCH = 12.5; // 12.5; // deg
    public static final double BALANCE_SPEED = 0.35; // m / s

    public static final PathConstraints CONSTRAINTS =
        new PathConstraints(MAX_SPEED / 1.8, MAX_ACCEL / 1.2);
  }

  public static final class SwerveModule {
    public static final class Driving {
      public static final MotorConfig MOTOR =
          MotorConfig.base().withNeutralBehavior(NeutralBehavior.BRAKE).withCurrentLimit(50);

      public static final Conversion CONVERSION =
          Conversion.base()
              .multiplyRadius(0.0381)
              .withUnits(Conversion.Units.RADIANS)
              .divideGearing(45.0)
              .divideGearing(22.0)
              .multiplyGearing(15.0)
              .multiplyGearing(14.0); // pinion teeth

      public static final PIDConstants PID = new PIDConstants(0.11, 0, 0.06);
      public static final SystemConstants FF = new SystemConstants(0.3, 2.7, 0.25);
    }

    public static final class Turning {
      public static final MotorConfig MOTOR =
          MotorConfig.base().withNeutralBehavior(NeutralBehavior.BRAKE).withCurrentLimit(20);

      public static final Conversion CONVERSION =
          Conversion.base()
              .withUnits(Conversion.Units.RADIANS)
              .withPulsesPerRev(PulsesPerRev.REV_INTEGRATED);

      public static final boolean ENCODER_INVERTED = true;

      public static final PIDConstants PID = new PIDConstants(2, 0, 0.1);
      // system constants only used in simulation
      public static final SystemConstants FF = new SystemConstants(0, 0.25, 0.015);
    }
  }

  public static final class Positions {

    public static final PlacementState INITIAL =
        PlacementState.fromRelative(Elevator.ZERO_OFFSET, Elbow.ELBOW_OFFSET, Math.PI);
    public static final PlacementState STOW =
        PlacementState.fromRelative(0, 1.21834, Math.PI / 2.0);

    // LOWEST COG
    public static final PlacementState BALANCE =
        PlacementState.fromRelative(Elevator.ZERO_OFFSET, Elbow.ELBOW_OFFSET + 0.1, 3);

    public static final PlacementState PASS_OLD =
        PlacementState.fromAbsolute(0, Math.PI / 2.0, Math.PI / 2.0);
    public static final PlacementState PASS_TO_BACK =
        PlacementState.fromAbsolute(0.03, Math.PI / 2.0, Math.PI);
    public static final PlacementState PASS_TO_FRONT =
        PlacementState.fromAbsolute(0.05, Math.PI / 2.0, Math.PI / 4.0);

    public static final PlacementState FRONT_INTAKE =
        PlacementState.fromAbsolute(0.454, -0.983, -0.055);
    public static final PlacementState BACK_INTAKE = STOW; // TODO

    public static final PlacementState FRONT_SINGLE_SUBSTATION_CONE =
        PlacementState.fromAbsolute(0.425006, 0.128855, -0.305);
    public static final PlacementState FRONT_SINGLE_SUBSTATION_CUBE =
        PlacementState.fromAbsolute(0.543571, -0.367516, 0.445646);
    public static final PlacementState BACK_DOUBLE_SUBSTATION =
        PlacementState.fromAbsolute(0, 2.8, Math.PI);

    public static final PlacementState FRONT_MID_CONE =
        PlacementState.fromAbsolute(0.061612, 0.493303, 0.001378);

    public static final PlacementState BACK_MID_CONE = STOW; // TODO
    public static final PlacementState BACK_HIGH_CONE =
        PlacementState.fromAbsolute(0.253, 3.072, 2.5);
    // ele 0.2475
    public static final PlacementState FRONT_MID_CUBE =
        PlacementState.fromAbsolute(0.11362, 0.458149, 0.353288);
    public static final PlacementState FRONT_HIGH_CUBE =
        PlacementState.fromAbsolute(0.113502, 0.333258, 0.353208);

    public static final PlacementState BACK_MID_CUBE = FRONT_MID_CUBE; // TODO
    public static final PlacementState BACK_HIGH_CUBE =
        PlacementState.fromAbsolute(0.245, 2.75, 3.17);
  }

  public static final class Auto {
    public static final double CUBE_OUTTAKE_TIME = 0.5; // seconds
    public static final double CONE_OUTTAKE_TIME = 3; // seconds
    public static final double INITIAL_INTAKE_TIME = 0.3; // seconds
    public static final double MOVING_INTAKE_TIME = 4; // seconds
  }

  public static final class Field {
    // public static final Map<Integer, Translation2d> INTAKE_POINTS =
    //     Map.ofEntries(
    //         Map.entry(1, new Translation2d(5, 2)),
    //         Map.entry(2, new Translation2d(5, 3)),
    //         Map.entry(3, new Translation2d(5, 4)),
    //         Map.entry(4, new Translation2d(5, 5)));
    public static final double FIELD_WIDTH_METERS = 8.02;
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
