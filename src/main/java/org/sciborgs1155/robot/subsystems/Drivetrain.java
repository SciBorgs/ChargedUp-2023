package org.sciborgs1155.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import java.util.Arrays;
import org.sciborgs1155.robot.Constants.DriveConstants;
import org.sciborgs1155.robot.Ports.DrivePorts;
import org.sciborgs1155.robot.subsystems.modules.SwerveModule;

public class Drivetrain extends SubsystemBase implements Loggable {
  @Log
  private final SwerveModule frontLeft =
      SwerveModule.create(
          DrivePorts.frontLeftDriveMotorPort,
          DrivePorts.frontLeftTurningMotorPort,
          DriveConstants.ANGULAR_OFFSETS[0]);

  @Log
  private final SwerveModule frontRight =
      SwerveModule.create(
          DrivePorts.frontRightDriveMotorPort,
          DrivePorts.frontRightTurningMotorPort,
          DriveConstants.ANGULAR_OFFSETS[1]);

  @Log
  private final SwerveModule rearLeft =
      SwerveModule.create(
          DrivePorts.rearLeftDriveMotorPort,
          DrivePorts.rearLeftTurningMotorPort,
          DriveConstants.ANGULAR_OFFSETS[2]);

  @Log
  private final SwerveModule rearRight =
      SwerveModule.create(
          DrivePorts.rearRightDriveMotorPort,
          DrivePorts.rearRightTurningMotorPort,
          DriveConstants.ANGULAR_OFFSETS[3]);

  private final SwerveModule[] modules = {frontLeft, frontRight, rearLeft, rearRight};

  // The gyro sensor
  // private final WPI_PigeonIMU gyro = new WPI_PigeonIMU(Sensors.PIGEON);

  // Odometry class for tracking robot pose
  private final SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(
          // DriveConstants.KINEMATICS, gyro.getRotation2d(), getModulePositions());
          DriveConstants.KINEMATICS, new Rotation2d(), getModulePositions());

  @Log private final Field2d field2d = new Field2d();

  private final FieldObject2d[] modules2d = new FieldObject2d[modules.length];

  public Drivetrain() {
    for (int i = 0; i < modules2d.length; i++) {
      modules2d[i] = field2d.getObject("module-" + i);
    }
  }
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    // odometry.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
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
    // scale inputs based on maximum values
    xSpeed *= DriveConstants.MAX_SPEED;
    ySpeed *= DriveConstants.MAX_SPEED;
    rot *= DriveConstants.MAX_ANGULAR_SPEED;

    var states =
        DriveConstants.KINEMATICS.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, new Rotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));

    // System.out.println(states[2].speedMetersPerSecond);
    setModuleStates(states);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    if (desiredStates.length != modules.length) {
      throw new IllegalArgumentException("desiredStates must have the same length as modules");
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_SPEED);
    for (int i = 0; i < modules.length; i++) modules[i].setDesiredState(desiredStates[i]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    Arrays.stream(modules).forEach(module -> module.resetEncoders());
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    // gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * <p>This not for internal use, as all internal angle values should be in radians.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  @Log
  public double getHeading() {
    // return gyro.getRotation2d().getDegrees();
    return 0;
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
  @Log
  public double getTurnRate() {
    // return gyro.getRate() * (DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
    return 0;
  }

  /**
   * Returns the pitch value recorded by the pigeon.
   *
   * @return The pitch value of the pigeon.
   */
  @Log
  public double getPitch() {
    // return gyro.getPitch();
    return 0;
  }

  @Override
  public void periodic() {
    // for (int i = 0; i < modules.length; i++) modules[i].setDesiredState(setpoint[i]);
    odometry.update(new Rotation2d(), getModulePositions());
    field2d.setRobotPose(getPose());
    for (int i = 0; i < modules2d.length; i++) {
      var transform =
          new Transform2d(DriveConstants.MODULE_OFFSET[i], modules[i].getPosition().angle);
      modules2d[i].setPose(getPose().transformBy(transform));
    }
  }

  @Override
  public void simulationPeriodic() {
    // gyro.getSimCollection()
    //     .addHeading(
    //         Units.radiansToDegrees(
    //
    // DriveConstants.KINEMATICS.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond
    //                 * 0.02));
  }
}
