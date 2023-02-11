package org.sciborgs1155.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import java.util.List;
import org.sciborgs1155.lib.MotorConfig;
import org.sciborgs1155.lib.MotorConfig.NeutralBehavior;

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

  public static final double RATE = 0.02; // roborio tickrate
  public static final double CONTROLLER_RATE = 0.015; // controller tickrate
  public static final double DEADBAND = 0.06;

  public static final String CAMERA_NAME = "photonvision";

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
        MotorConfig.base().withNeutralBehavior(NeutralBehavior.BRAKE);

    public static final MotorConfig ELBOW =
        MotorConfig.base().withNeutralBehavior(NeutralBehavior.BRAKE).withCurrentLimit(50);

    public static final MotorConfig WRIST =
        MotorConfig.base().withNeutralBehavior(NeutralBehavior.BRAKE);

    public static final MotorConfig INTAKE =
        MotorConfig.base().withNeutralBehavior(NeutralBehavior.BRAKE);
  }

  public static final class Dimensions {
    // no clue what the actual constants are
    public static final double ELEVATOR_MIN_HEIGHT = 0;
    public static final double ELEVATOR_MAX_HEIGHT = Units.inchesToMeters(49.3);
    public static final double FOREARM_LENGTH = Units.inchesToMeters(41);
    public static final double CLAW_LENGTH = Units.inchesToMeters(20);

    public static final double ELBOW_MIN_ANGLE = Units.degreesToRadians(-90);
    public static final double ELBOW_MAX_ANGLE = Units.degreesToRadians(270);
    public static final double WRIST_MIN_ANGLE = Units.degreesToRadians(-90);
    public static final double WRIST_MAX_ANGLE = Units.degreesToRadians(90);

    public static final double TRACK_WIDTH = Units.inchesToMeters(17);
    // Distance between centers of right and left wheels on robot
    public static final double WHEEL_BASE = Units.inchesToMeters(17);
    // Distance between front and back wheels on robot

    public static final Translation3d CAM_TRANSLATION = new Translation3d();
    public static final Rotation3d CAM_ROTATION = new Rotation3d();
    public static final Transform3d ROBOT_TO_CAM = new Transform3d(CAM_TRANSLATION, CAM_ROTATION);

    public static final double FIELD_LENGTH = Units.feetToMeters(54);
    public static final double FIELD_WIDTH = Units.feetToMeters(27);

    // test tag at 0
    public static final AprilTag TEST_TAG_0 = new AprilTag(0, new Pose3d());
    public static final List<AprilTag> TEST_TAGS = List.of(TEST_TAG_0);
  }

  public static final class Arm {
    public static final class Wrist {
      // not actual values
      public static final double kP = 0;
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
      public static final double kP = 10;
      public static final double kI = 0;
      public static final double kD = 0;

      public static final double kS = 0;
      public static final double kG = 10;
      public static final double kV = 0;
      public static final double kA = 0;

      public static final double MAX_VELOCITY = 3; // radians / s
      public static final double MAX_ACCEL = 3; // radians / s^2
      public static final TrapezoidProfile.Constraints CONSTRAINTS =
          new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCEL);

      public static final double GEAR_RATIO = 1 / 6.0;
      public static final double MOVEMENT_PER_SPIN = (1.5 * Math.PI);
    }
  }

  public static final class Intake {
    public static final double WHEEL_SPEED = 0.6;
  }

  public static final class Elevator {
    public static final double MAX_SPEED = 4; // m/s
    public static final double MAX_ACCEL = 3; // m/s^2
    public static final double kP = 5;
    public static final double kI = 0;
    public static final double kD = 0.1;

    public static final double kS = 0;
    public static final double kG = 0.762;
    public static final double kV = 0.762;
    public static final double kA = 0;
    public static final TrapezoidProfile.Constraints CONSTRAINTS =
        new TrapezoidProfile.Constraints(MAX_SPEED, MAX_ACCEL);
    public static final double GEAR_RATIO = 1.0;
    public static final double MOVEMENTPERSPIN =
        1.1938 / 6.0; // m/ (50 rotations of motor) wtf is this
  }

  public static final class DriveConstants {
    public static final double MAX_SPEED = 7; // m / s
    public static final double MAX_ANGULAR_SPEED = 4 * Math.PI; // rad / s

    public static final Translation2d[] MODULE_OFFSET = {
      new Translation2d(Dimensions.WHEEL_BASE / 2, Dimensions.TRACK_WIDTH / 2), // front left
      new Translation2d(Dimensions.WHEEL_BASE / 2, -Dimensions.TRACK_WIDTH / 2), // front right
      new Translation2d(-Dimensions.WHEEL_BASE / 2, Dimensions.TRACK_WIDTH / 2), // rear left
      new Translation2d(-Dimensions.WHEEL_BASE / 2, -Dimensions.TRACK_WIDTH / 2) // rear right
    };

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(MODULE_OFFSET);

    public static final boolean GYRO_REVERSED = false;

    // angular offsets of the modules, since we use absolute encoders
    // ignored (used as 0) in simulation because that's how sim works
    public static final double[] ANGULAR_OFFSETS = {
      -Math.PI / 2, // front left
      0, // front right
      Math.PI, // rear left
      Math.PI / 2 // rear right
    };
  }

  public static final class ModuleConstants {
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
      public static final double ENCODER_VELOCITY_FACTOR = ENCODER_POSITION_FACTOR / 60.0; // m / s
      // public static final double ENCODER_VELOCITY_FACTOR = 1;

      public static final double P = 0.07;
      public static final double I = 0;
      public static final double D = 0.06;

      public static final double S = 0.27;
      public static final double V = 0.4;
      public static final double A = 0.2;
    }

    public static final class Turning {
      public static final double ENCODER_POSITION_FACTOR = (2 * Math.PI); // rad
      public static final double ENCODER_VELOCITY_FACTOR =
          ENCODER_POSITION_FACTOR / 60.0; // rad / s
      public static final boolean ENCODER_INVERTED = true;

      public static final double MAX_ANGULAR_SPEED = 2 * Math.PI; // rad / s
      public static final double MAX_ANGULAR_ACCELERATION = 2 * Math.PI; // rad / s^2

      public static final double P = 1.7;
      public static final double I = 0;
      public static final double D = 0.02;

      // feedforward constants for simulation
      public static final double S = 0.1;
      public static final double V = 0.1;
      public static final double A = 0.1;

      // pid wrapping
      public static final double MIN_INPUT = 0;
      public static final double MAX_INPUT = ENCODER_POSITION_FACTOR;
    }
  }

  public static final class AutoConstants {
    public static final double MAX_SPEED = DriveConstants.MAX_SPEED; // m/s
    public static final double MAX_ACCEL = 4; // m/s^2
    public static final double MAX_ANG_SPEED = 1.5 * DriveConstants.MAX_ANGULAR_SPEED; // rad/s
    public static final double MAX_ANG_ACCEL = Math.PI; // rad/s^2

    public static final double P_X_CONTROLLER = 2;
    public static final double P_Y_CONTROLLER = 2;
    public static final double P_THETA_CONTROLLER = 4;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
        new TrapezoidProfile.Constraints(MAX_ANG_SPEED, MAX_ANG_ACCEL);
  }
}
