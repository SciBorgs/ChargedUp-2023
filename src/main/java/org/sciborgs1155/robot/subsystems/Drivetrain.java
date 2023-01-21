package org.sciborgs1155.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.DriveConstants;
import org.sciborgs1155.robot.Ports.DrivePorts;
import org.sciborgs1155.robot.Ports.Sensors;
import org.sciborgs1155.robot.subsystems.modules.SwerveModule;

public class Drivetrain extends SubsystemBase implements Loggable {
  @Log
  private final SwerveModule frontLeft =
      SwerveModule.create(
          DrivePorts.FRONT_LEFT_DRIVE,
          DrivePorts.FRONT_LEFT_TURNING,
          DriveConstants.ANGULAR_OFFSETS[0]);

  @Log
  private final SwerveModule frontRight =
      SwerveModule.create(
          DrivePorts.FRONT_RIGHT_DRIVE,
          DrivePorts.FRONT_RIGHT_TURNING,
          DriveConstants.ANGULAR_OFFSETS[1]);

  @Log
  private final SwerveModule rearLeft =
      SwerveModule.create(
          DrivePorts.REAR_LEFT_DRIVE,
          DrivePorts.REAR_LEFT_TURNING,
          DriveConstants.ANGULAR_OFFSETS[2]);

  @Log
  private final SwerveModule rearRight =
      SwerveModule.create(
          DrivePorts.REAR_RIGHT_DRIVE,
          DrivePorts.REAR_RIGHT_TURNING,
          DriveConstants.ANGULAR_OFFSETS[3]);

  private final SwerveModule[] modules = {frontLeft, frontRight, rearLeft, rearRight};

  // The gyro sensor
  @Log private final WPI_PigeonIMU gyro = new WPI_PigeonIMU(Sensors.PIGEON);

  // Odometry class for tracking robot pose
  private final SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(
          DriveConstants.KINEMATICS, gyro.getRotation2d(), getModulePositions());

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
    odometry.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
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
    // deadband
    if (Math.abs(xSpeed) < 0.05) xSpeed = 0;
    if (Math.abs(ySpeed) < 0.05) ySpeed = 0;
    if (Math.abs(rot) < 0.005) rot = 0;

    // System.out.println(ySpeed);
    // scale inputs based on maximum values
    xSpeed *= DriveConstants.MAX_SPEED;
    ySpeed *= DriveConstants.MAX_SPEED;
    rot *= DriveConstants.MAX_ANGULAR_SPEED;

    var states =
        DriveConstants.KINEMATICS.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));

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
    for (int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(desiredStates[i]);
    }
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    Arrays.stream(modules).forEach(SwerveModule::resetEncoders);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.reset();
  }

  private SwerveModuleState[] getModuleStates() {
    return Arrays.stream(modules).map(SwerveModule::getState).toArray(SwerveModuleState[]::new);
  }

  private SwerveModulePosition[] getModulePositions() {
    return Arrays.stream(modules)
        .map(SwerveModule::getPosition)
        .toArray(SwerveModulePosition[]::new);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  @Log
  public double getTurnRate() {
    return gyro.getRate() * (DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
  }

  /**
   * Returns the pitch value recorded by the pigeon.
   *
   * @return The pitch value of the pigeon.
   */
  @Log
  public double getPitch() {
    return gyro.getPitch();
  }

  @Override
  public void periodic() {
    // for (int i = 0; i < modules.length; i++) modules[i].setDesiredState(setpoint[i]);
    odometry.update(gyro.getRotation2d(), getModulePositions());
    field2d.setRobotPose(getPose());

    for (int i = 0; i < modules2d.length; i++) {
      var transform =
          new Transform2d(DriveConstants.MODULE_OFFSET[i], modules[i].getPosition().angle);
      modules2d[i].setPose(getPose().transformBy(transform));
    }
  }

  @Override
  public void simulationPeriodic() {
    gyro.getSimCollection()
        .addHeading(
            Units.radiansToDegrees(
                DriveConstants.KINEMATICS.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond
                    * Constants.RATE));
  }

  public Command drive(
      DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rot, boolean fieldRelative) {
    return this.run(
        () -> drive(xSpeed.getAsDouble(), ySpeed.getAsDouble(), rot.getAsDouble(), fieldRelative));
  }

  /** Sets the drivetrain to an "X" configuration, preventing movement */
  public Command lock() {
    var states =
        new SwerveModuleState[] {
          new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(45))
        };
    return this.runOnce(() -> setModuleStates(states));
  }
}
