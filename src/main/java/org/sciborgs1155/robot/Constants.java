package org.sciborgs1155.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
 *   <li>time: seconds
 *   <li>angle: radians
 * </ul>
 */
public final class Constants {
  public static final class Motors {
    public static final MotorConfig moduleDriveConfig =
        MotorConfig.base().setNeutralBehavior(NeutralBehavior.BRAKE).setCurrentLimit(50);

    public static final MotorConfig moduleTurnConfig =
        MotorConfig.base()
            .setInverted(true)
            .setNeutralBehavior(NeutralBehavior.BRAKE)
            .setCurrentLimit(20);
  }

  public static final class DriveConstants {

    public static final double maxSpeed = 4.8; // m / s
    public static final double maxAngularSpeed = 2 * Math.PI; // rad / s

    public static final double trackWidth = 0.5;
    // Distance between centers of right and left wheels on robot
    public static final double wheelBase = 0.7;
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2, trackWidth / 2),
            new Translation2d(wheelBase / 2, -trackWidth / 2),
            new Translation2d(-wheelBase / 2, trackWidth / 2),
            new Translation2d(-wheelBase / 2, -trackWidth / 2));

    public static final boolean gyroReversed = false;

    // angular offsets of the modules, since we use absolute encoders
    public static final double frontLeftAngularOffset = -Math.PI / 2;
    public static final double frontRightAngularOffset = 0;
    public static final double backLeftAngularOffset = Math.PI;
    public static final double backRightAngularOffset = Math.PI / 2;
  }

  public static final class ModuleConstants {

    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = 5676.0 / 60.0;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the
    // bevel pinion
    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

    public final class Driving {
      public static final double encoderPositionFactor =
          (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction;
      public static final double encoderVelocityFactor = encoderPositionFactor / 60.0;

      public static final double kP = 0.04;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double kV = 1 / kDriveWheelFreeSpeedRps; // ??

      public static final double minOutput = -1;
      public static final double maxOutput = 1;
    }

    public final class Turning {
      public static final double encoderPositionFactor = (2 * Math.PI);
      public static final double encoderVelocityFactor = encoderPositionFactor / 60.0;

      public static final double kP = 1;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double kV = 0;

      public static final double minInput = 0;
      public static final double maxInput = encoderPositionFactor;
      public static final double minOutput = -1;
      public static final double maxOutput = 1;
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
