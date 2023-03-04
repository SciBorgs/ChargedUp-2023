package org.sciborgs1155.robot;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.sciborgs1155.lib.PlacementState;
import org.sciborgs1155.lib.constants.MotorConfig;
import org.sciborgs1155.lib.constants.MotorConfig.NeutralBehavior;
import org.sciborgs1155.lib.constants.PIDConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity. <b>Units</b>
 *
 * <ul>
 *   <li>length: meters
 *   <li>time: seconds, minutes
 *   <li>angle: radians
 * </ul>
 */
public final class Constants {

  public static final double RATE = 0.02; // roborio tickrate (s)
  public static final double DEADBAND = 0.1;
  public static final int THROUGH_BORE_CPP = 2048;

  public static final class Motors {
    public static final MotorConfig DRIVE =
        MotorConfig.base()
            .withBurnFlash(false)
            .withNeutralBehavior(NeutralBehavior.BRAKE)
            .withCurrentLimit(50);

    public static final MotorConfig TURN =
        MotorConfig.base()
            .withBurnFlash(false)
            .withNeutralBehavior(NeutralBehavior.BRAKE)
            .withCurrentLimit(20);

    public static final MotorConfig ELEVATOR =
        MotorConfig.base()
            .withBurnFlash(false)
            .withNeutralBehavior(NeutralBehavior.BRAKE)
            .withCurrentLimit(40);

    public static final MotorConfig ELBOW =
        MotorConfig.base()
            .withBurnFlash(false)
            .withNeutralBehavior(NeutralBehavior.BRAKE)
            .withCurrentLimit(50);

    public static final MotorConfig WRIST =
        MotorConfig.base().withNeutralBehavior(NeutralBehavior.BRAKE);

    public static final MotorConfig INTAKE =
        MotorConfig.base().withNeutralBehavior(NeutralBehavior.BRAKE);
  }

  public static final class Dimensions {
    public static final double ELEVATOR_MIN_HEIGHT = 0;
    public static final double ELEVATOR_MAX_HEIGHT = 0.7; // m

    public static final double ELBOW_MIN_ANGLE = -Math.PI / 2.0;
    public static final double ELBOW_MAX_ANGLE = 3.0 * Math.PI / 2.0;
    public static final double WRIST_MIN_ANGLE = -Math.PI;
    public static final double WRIST_MAX_ANGLE = Math.PI;

    public static final double CLAW_LENGTH = Units.inchesToMeters(20);
    public static final double CLAW_MASS = 3.6; // var used to say 4.4, also could be 3.62874
    public static final double CLAW_MOI =
        1.0 / 12.0 * CLAW_MASS * CLAW_LENGTH * CLAW_LENGTH; // moi about center point
    public static final double CLAW_RADIUS = CLAW_LENGTH / 2.0;

    public static final double FOREARM_LENGTH = Units.inchesToMeters(41);
    public static final double FOREARM_MASS = Units.lbsToKilograms(4.2);
    public static final double FOREARM_MOI =
        1. / 12 * FOREARM_MASS * FOREARM_LENGTH * FOREARM_LENGTH;
    public static final double FOREARM_RADIUS = FOREARM_LENGTH / 2;

    public static final double ARM_LENGTH = CLAW_LENGTH + FOREARM_LENGTH;

    public static final double ELEVATOR_MASS = 4;

    public static final double TRACK_WIDTH = Units.inchesToMeters(24);
    // Distance between centers of right and left wheels on robot
    public static final double WHEEL_BASE = Units.inchesToMeters(24);
    // Distance between front and back wheels on robot

    // Field dimensions
    public static final double FIELD_LENGTH = Units.feetToMeters(54);
    public static final double FIELD_WIDTH = Units.feetToMeters(27);
  }

  public static final class Vision {
    // Camera names
    public static final String FRONT_CAM = "frontPhotonVision";
    public static final String BACK_CAM = "backPhotonVision";

    // Robot to camera translations
    public static final Translation3d FRONT_CAM_TRANSLATION = new Translation3d();
    public static final Rotation3d FRONT_CAM_ROTATION = new Rotation3d();
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
      public static final double kP = 0.6;
      public static final double kI = 0;
      public static final double kD = 0;

      public static final double kS = 0;
      public static final double kG = 0;
      public static final double kV = 0;
      public static final double kA = 0;

      public static final double MAX_VELOCITY = 3; // radians / s
      public static final double MAX_ACCEL = 3; // radians / s^2
      public static final TrapezoidProfile.Constraints CONSTRAINTS =
          new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCEL);
    }

    public static final class Elbow {
      public static final double GEARING = 12.0 / 72.0; // rot
      public static final double CONVERSION = GEARING * 2.0 * Math.PI; // rad
      public static final double ENCODER_FACTOR = CONVERSION / Constants.THROUGH_BORE_CPP;

      // public static final double kP = 8.0252;
      public static final double kP = 2;
      public static final double kI = 0;
      public static final double kD = 0.2;

      public static final double kS = 0.12117;
      public static final double kG = 0.48397;
      public static final double kV = 1.1967;
      public static final double kA = 0.23693;

      public static final double MAX_VELOCITY = 1; // rad / s
      public static final double MAX_ACCEL = 0.5; // rad / s^2
      public static final TrapezoidProfile.Constraints CONSTRAINTS =
          new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCEL);
    }
  }

  public static final class Elevator {
    public static final double SPROCKET_RADIUS = Units.inchesToMeters(0.716);
    public static final double CONVERSION = 2.0 * Math.PI * SPROCKET_RADIUS; // m
    public static final double ENCODER_FACTOR = CONVERSION / Constants.THROUGH_BORE_CPP;

    public static final double MAX_SPEED = 20; // m/s
    public static final double MAX_ACCEL = 8; // m/s^2
    public static final double kP = 45; // 384.48
    public static final double kI = 0;
    public static final double kD = 1; // 47.131

    public static final double kS = 0.39974;
    public static final double kG = 0.060732;
    public static final double kV = 29.798;
    public static final double kA = 2.554;
    public static final TrapezoidProfile.Constraints CONSTRAINTS =
        new TrapezoidProfile.Constraints(MAX_SPEED, MAX_ACCEL);
  }

  public static final class Intake {
    public static final double WHEEL_SPEED = 0.4;
  }

  public static final class Drive {
    public static final double MAX_SPEED = 7; // m / s
    public static final double MAX_ANGULAR_SPEED = 2 * Math.PI; // rad / s

    public static final Translation2d[] MODULE_OFFSET = {
      new Translation2d(Dimensions.WHEEL_BASE / 2, Dimensions.TRACK_WIDTH / 2), // front left
      new Translation2d(Dimensions.WHEEL_BASE / 2, -Dimensions.TRACK_WIDTH / 2), // front right
      new Translation2d(-Dimensions.WHEEL_BASE / 2, Dimensions.TRACK_WIDTH / 2), // rear left
      new Translation2d(-Dimensions.WHEEL_BASE / 2, -Dimensions.TRACK_WIDTH / 2) // rear right
    };

    // angular offsets of the modules, since we use absolute encoders
    // ignored (used as 0) in simulation because the simulated robot doesn't have offsets
    public static final double[] ANGULAR_OFFSETS = {
      -Math.PI / 2, // front left
      0, // front right
      Math.PI, // rear left
      Math.PI / 2 // rear right
    };

    public static final double MAX_RATE = 12; // m / s^2
    // limits open loop accel, used to prevent tipping
  }

  public static final class SwerveModule {
    // we use a 14T pinion
    public static final int PINION_TEETH = 14;

    public static final double WHEEL_DIAMETER = 0.0762;

    // 45 teeth on the wheel's bevel gear
    // 22 teeth on the first-stage spur gear
    // 15 teeth on the bevel pinion
    public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (PINION_TEETH * 15);

    public static final class Driving {
      public static final double ENCODER_POSITION_FACTOR =
          (WHEEL_DIAMETER * Math.PI) / DRIVING_MOTOR_REDUCTION; // m
      public static final double ENCODER_VELOCITY_FACTOR = ENCODER_POSITION_FACTOR / 60.0; // m/s

      public static final PIDConstants PID = new PIDConstants(0.07, 0, 0.06);

      public static final double kS = 0.27;
      public static final double kV = 0.4;
      public static final double kA = 0.2;
    }

    public static final class Turning {
      public static final double ENCODER_POSITION_FACTOR = (2 * Math.PI); // rad
      public static final double ENCODER_VELOCITY_FACTOR =
          ENCODER_POSITION_FACTOR / 60.0; // rad / s
      public static final boolean ENCODER_INVERTED = true;

      public static final double MAX_ANGULAR_SPEED = 2 * Math.PI; // rad / s
      public static final double MAX_ANGULAR_ACCELERATION = 2 * Math.PI; // rad / s^2

      public static final PIDConstants PID = new PIDConstants(1.7, 0, 0.1);

      // feedforward constants for simulation
      public static final double kS = 0;
      public static final double kV = 0.25;
      public static final double kA = 0.015;

      // pid wrapping
      public static final double MIN_INPUT = 0;
      public static final double MAX_INPUT = ENCODER_POSITION_FACTOR;
    }
  }

  public static final class Positions {
    // tested

    // untested
    public static final PlacementState SAFE = PlacementState.fromAbsolute(0, Math.PI / 2.0, 0);

    public static final PlacementState FRONT_MID_CONE = PlacementState.fromAbsolute(0, 0.2, 0.6);
    public static final PlacementState FRONT_HIGH_CONE = PlacementState.fromAbsolute(0, 1, 1.1);

    public static final PlacementState BACK_LOW_CONE = PlacementState.fromAbsolute(0, 0.1, 0);
    public static final PlacementState BACK_MID_CONE = PlacementState.fromAbsolute(0, 0.2, 0.6);
    public static final PlacementState BACK_HIGH_CONE = PlacementState.fromAbsolute(0, 1, 1.1);

    public static final PlacementState FRONT_MID_CUBE = PlacementState.fromAbsolute(0, 0.2, 0.6);
    public static final PlacementState FRONT_HIGH_CUBE = PlacementState.fromAbsolute(0, 1, 1.1);

    public static final PlacementState BACK_LOW_CUBE = PlacementState.fromAbsolute(0, 0.1, 0);
    public static final PlacementState BACK_MID_CUBE = PlacementState.fromAbsolute(0, 0.2, 0.6);
    public static final PlacementState BACK_HIGH_CUBE = PlacementState.fromAbsolute(0, 1, 1.1);
  }

  public static final class Auto {
    public static final class Cartesian {
      public static final double kP = 1.2;
      public static final double kI = 0;
      public static final double kD = 0;
    }

    public static final class Angular {
      public static final double kP = 1.2;
      public static final double kI = 0;
      public static final double kD = 1;
    }

    public static final double MAX_SPEED = Drive.MAX_SPEED; // m/s
    public static final double MAX_ACCEL = 4; // m/s^2
    public static final PathConstraints CONSTRAINTS = new PathConstraints(RATE, RATE);
  }
}
