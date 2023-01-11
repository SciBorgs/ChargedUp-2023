package org.sciborgs1155.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import java.util.Arrays;
import org.sciborgs1155.robot.Constants.DriveConstants;
import org.sciborgs1155.robot.Ports.Drivetrain;

public class DriveSubsystem extends SubsystemBase implements Loggable {
  @Log
  private final SwerveModule frontLeft =
      new SwerveModule(
          Drivetrain.frontLeftDriveMotorPort,
          Drivetrain.frontLeftTurningMotorPort,
          DriveConstants.frontDriveConfig,
          DriveConstants.frontTurnConfig);

  @Log
  private final SwerveModule rearLeft =
      new SwerveModule(
          Drivetrain.rearLeftDriveMotorPort,
          Drivetrain.rearLeftTurningMotorPort,
          DriveConstants.rearDriveConfig,
          DriveConstants.rearTurnConfig);

  @Log
  private final SwerveModule frontRight =
      new SwerveModule(
          Drivetrain.frontRightDriveMotorPort,
          Drivetrain.frontRightTurningMotorPort,
          DriveConstants.frontDriveConfig,
          DriveConstants.frontTurnConfig);

  @Log
  private final SwerveModule rearRight =
      new SwerveModule(
          Drivetrain.rearRightDriveMotorPort,
          Drivetrain.rearRightTurningMotorPort,
          DriveConstants.rearDriveConfig,
          DriveConstants.rearTurnConfig);

  private final SwerveModule[] modules = {frontLeft, rearLeft, frontRight, rearRight};

  // The gyro sensor
  private final WPI_PigeonIMU m_gyro = new WPI_PigeonIMU(2);

  // Odometry class for tracking robot pose
  private SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics, m_gyro.getRotation2d(), getModulePositions());

  @Log private Field2d field2d = new Field2d();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {}

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getRotation2d(), getModulePositions(), pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    setModuleStates(
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    Arrays.stream(modules).forEach(module -> module.resetEncoders());
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  private SwerveModuleState[] getModuleStates() {
    return Arrays.stream(modules)
        .map(module -> module.getState())
        .toArray(SwerveModuleState[]::new);
  }

  private SwerveModulePosition[] getModulePositions() {
    return Arrays.stream(modules)
        .map(module -> module.getPosition())
        .toArray(SwerveModulePosition[]::new);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  } // SpeedLimit is just to make sure I dont burn down the school

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(m_gyro.getRotation2d(), getModulePositions());
    field2d.setRobotPose(getPose());
  }
}
