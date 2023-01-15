package org.sciborgs1155.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import java.util.Arrays;
import org.sciborgs1155.robot.Constants.DriveConstants;
import org.sciborgs1155.robot.Ports.DrivePorts;
import org.sciborgs1155.robot.Ports.Sensors;


public class Drivetrain extends SubsystemBase implements Loggable {
  @Log
  private final SwerveModule frontLeft = new SwerveModule(
      DrivePorts.frontLeftDriveMotorPort,
      DrivePorts.frontLeftTurningMotorPort,
      DriveConstants.frontLeftAngularOffset);

  @Log
  private final SwerveModule frontRight = new SwerveModule(
      DrivePorts.frontRightDriveMotorPort,
      DrivePorts.frontRightTurningMotorPort,
      DriveConstants.frontRightAngularOffset);

  @Log
  private final SwerveModule rearLeft = new SwerveModule(
      DrivePorts.rearLeftDriveMotorPort,
      DrivePorts.rearLeftTurningMotorPort,
      DriveConstants.backLeftAngularOffset);

  @Log
  private final SwerveModule rearRight = new SwerveModule(
      DrivePorts.rearRightDriveMotorPort,
      DrivePorts.rearRightTurningMotorPort,
      DriveConstants.backRightAngularOffset);

  private final SwerveModule[] modules = { frontLeft, frontRight, rearLeft, rearRight };

  // gyro sensor
  private final WPI_PigeonIMU gyro = new WPI_PigeonIMU(Sensors.PIGEON);

  // Odometry class for tracking robot pose
  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
      DriveConstants.kinematics, gyro.getRotation2d(), getModulePositions());

  @Log
  private final Field2d field2d = new Field2d();

  private final FieldObject2d[] modules2d = Arrays.stream(modules)
      .map(module -> field2d.getObject(module.getClass().getSimpleName()))
      .toArray(FieldObject2d[]::new);

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
    odometry.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // scale inputs based on maximum values
    // xSpeed *= DriveConstants.maxSpeed;
    // ySpeed *= DriveConstants.maxSpeed;
    // rot *= DriveConstants.maxAngularSpeed;

    var states = DriveConstants.kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));

    System.out.println(states[2].speedMetersPerSecond);
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

    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.maxSpeed);
    for (int i = 0; i < modules.length; i++)
      modules[i].setDesiredState(desiredStates[i]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    Arrays.stream(modules).forEach(module -> module.resetEncoders());
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * <p>
   * This not for internal use, as all internal angle values should be in radians.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  @Log
  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
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
    return gyro.getRate() * (DriveConstants.gyroReversed ? -1.0 : 1.0);
  }

  @Override
  public void periodic() {
    odometry.update(gyro.getRotation2d(), getModulePositions());
    field2d.setRobotPose(getPose());
  }

  // modules2d[i].setPose(getPose());
  // modules2d[i].setPose(
  // new Pose2d(
  // getPose().getTranslation().rotateBy(gyro.getRotation2d()),
  // modules[i].getState().angle.plus(gyro.getRotation2d())));
  @Override
  public void simulationPeriodic() {
    gyro.getSimCollection()
        .addHeading(
            Units.radiansToDegrees(
                DriveConstants.kinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond
                    * 0.02));
    // gyro.getSimCollection().addHeading(0);
  }
}
