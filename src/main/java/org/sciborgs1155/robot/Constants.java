package org.sciborgs1155.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import java.awt.Color;
import java.util.Map;
import java.util.function.Consumer;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.sciborgs1155.robot.subsystems.arm.ElevatorIO.ElevatorConfig;
import org.sciborgs1155.robot.subsystems.arm.JointIO.JointConfig;

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

  public static final class Wrist {
    public static final Consumer<CANSparkMax> MOTOR_CFG =
        spark -> {
          spark.restoreFactoryDefaults();
          spark.setCANTimeout(50);
          spark.setIdleMode(IdleMode.kBrake);
          spark.setOpenLoopRampRate(0);
          spark.setSmartCurrentLimit(80);
        };

    public static final double MOTOR_GEARING = 53.125 / 1.0;
    // Gearing for motor : angle (radians)

    public static final double CONVERSION_RELATIVE = 2.0 * Math.PI / THROUGHBORE_PPR;
    public static final double CONVERSION_ABS = -2.0 * Math.PI;

    public static final PIDConstants PID_NEW = new PIDConstants(5.5, 0, 0.1);
    public static final PIDConstants PID_OLD = new PIDConstants(5.5, 0, 0.1);
    public static final ArmFFConstants FF_NEW =
        new ArmFFConstants(0.34613, 0.25692, 0.78381, 0.090836);
    public static final ArmFFConstants FF_OLD = new ArmFFConstants(0.1542, 0.6, 0.91, 0.038046);

    public static final Constraints CONSTRAINTS = new Constraints(2.45, 3.6);

    public static final double MAX_ANGLE = Math.PI;
    public static final double MIN_ANGLE = -Math.PI;

    public static final double ZERO_OFFSET = 0.885; // duty cycle

    public static final JointConfig CONFIG_OLD =
        new JointConfig(
            FF_OLD,
            PID_OLD,
            DCMotor.getNEO(1),
            MOTOR_GEARING,
            Dimensions.CLAW_LENGTH_OLD,
            Dimensions.CLAW_MASS_OLD,
            MIN_ANGLE,
            MAX_ANGLE);
    public static final JointConfig CONFIG_NEW =
        new JointConfig(
            FF_NEW,
            PID_NEW,
            DCMotor.getNEO(1),
            MOTOR_GEARING,
            Dimensions.CLAW_LENGTH,
            Dimensions.CLAW_MASS,
            MIN_ANGLE,
            MAX_ANGLE);
  }

  public static final class Elbow {
    public static final Consumer<CANSparkMax> MOTOR_CFG =
        spark -> {
          spark.restoreFactoryDefaults();
          spark.setCANTimeout(50);
          spark.setIdleMode(IdleMode.kBrake);
          spark.setOpenLoopRampRate(0);
          spark.setSmartCurrentLimit(50);
        };

    public static final double MOTOR_GEARING = 63.75 / 1.0;

    public static final double ENCODER_RATIO = 12.0 / 72.0;
    public static final double CONVERSION = ENCODER_RATIO * 2 * Math.PI / THROUGHBORE_PPR;
    
    // TODO refactor this zoo of constants
    public static final PIDConstants PID_NEW = new PIDConstants(12, 0, 1.1); // d = 2.18954
    public static final PIDConstants PID_OLD = new PIDConstants(12, 0, 1.1); // d = 2.18954
    public static final ArmFFConstants FF_NEW =
        new ArmFFConstants(0.06403, 0.50715, 1.3482, 0.049377);
    public static final ArmFFConstants FF_OLD = new ArmFFConstants(0.020283, 0.71, 1.3174, 0.20891);

    public static final Constraints CONSTRAINTS = new Constraints(2.9, 3.4);

    public static final double MAX_ANGLE = 3.0 * Math.PI / 2.0;
    public static final double MIN_ANGLE = -Math.PI / 2.0;

    public static final JointConfig CONFIG_OLD =
        new JointConfig(
            FF_OLD,
            PID_OLD,
            DCMotor.getNEO(3),
            MOTOR_GEARING,
            Dimensions.FOREARM_LENGTH,
            Dimensions.FOREARM_MASS + Dimensions.CLAW_MASS_OLD,
            MIN_ANGLE,
            MAX_ANGLE);

    public static final JointConfig CONFIG_NEW =
        new JointConfig(
            FF_NEW,
            PID_NEW,
            DCMotor.getNEO(3),
            MOTOR_GEARING,
            Dimensions.FOREARM_LENGTH,
            Dimensions.FOREARM_MASS + Dimensions.CLAW_MASS,
            MIN_ANGLE,
            MAX_ANGLE);

    public static final double OFFSET = -1.248660;

    public static final double FAILING_DEBOUNCE_TIME = 0.2;
  }

  public static final class Elevator {
    public static final Consumer<CANSparkMax> MOTOR_CFG =
        spark -> {
          spark.restoreFactoryDefaults();
          spark.setCANTimeout(50);
          spark.setIdleMode(IdleMode.kBrake);
          spark.setOpenLoopRampRate(0);
          spark.setSmartCurrentLimit(35);
        };

    public static final double MOTOR_GEARING = 30.0 / 1.0;
    // Gearing for motor : height (meters)

    public static final double SPROCKET_RADIUS = 0.0181864;
    // Radius of the sprocket in meters

    public static final double SPROCKET_CIRCUMFERENCE = 2.0 * Math.PI * SPROCKET_RADIUS;
    // Circumference of the sprocket in meters (2 * π * R)

    public static final double CONVERSION_RELATIVE = SPROCKET_CIRCUMFERENCE / THROUGHBORE_PPR;

    public static final double kP = 50;
    public static final double kI = 0;
    public static final double kD = 1;

    public static final double s = 0.4;
    public static final double v = 0.069335;
    public static final double g = 33.25;
    public static final double a = 1.5514;
    ;

    public static final double MIN_HEIGHT = 0;
    public static final double MAX_HEIGHT = 0.65;

    public static final ElevatorConfig CONFIG =
        new ElevatorConfig(
            FF,
            PID,
            DCMotor.getNEO(3),
            MOTOR_GEARING,
            Dimensions.CARRIAGE_MASS + Dimensions.FOREARM_MASS + Dimensions.CLAW_MASS,
            SPROCKET_RADIUS,
            MIN_HEIGHT,
            MAX_HEIGHT);

    public static final int SAMPLE_SIZE_TAPS = 3;
    public static final int CURRENT_THRESHOLD = 35;

    public static final Constraints CONSTRAINTS = new Constraints(3, 3.35);

    public static final double ZERO_OFFSET = 0.61842;

    public static final double FAILING_DEBOUNCE_TIME = 0.2;
  }

  public static final class Intake {
    public static final Consumer<CANSparkMax> MOTOR_CFG =
        spark -> {
          spark.restoreFactoryDefaults();
          spark.setCANTimeout(50);
          spark.setInverted(true);
          spark.setIdleMode(IdleMode.kBrake);
          spark.setOpenLoopRampRate(0);
          spark.setSmartCurrentLimit(50);
        };

    public static final double DEFAULT_SPEED = 0.05;

    public static final double CONE_SPEED = 1;
    public static final double CUBE_SPEED = -0.4;

    public static final double THRESHOLD = 0.5;
  }

  public static final class Auto {
    public static final double CUBE_OUTTAKE_TIME = 0.5; // seconds
    public static final double CONE_OUTTAKE_TIME = 3; // seconds
    public static final double INITIAL_INTAKE_TIME = 0.3; // seconds
    public static final double MOVING_INTAKE_TIME = 4; // seconds

    public static final double BALANCE_kP = 0.05;
    public static final double BALANCE_kI = 0;
    public static final double BALANCE_kD = 0;

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
