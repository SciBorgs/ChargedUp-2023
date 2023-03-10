package org.sciborgs1155.robot.subsystems;

import static org.sciborgs1155.robot.Constants.Drive.*;
import static org.sciborgs1155.robot.Ports.Drive.*;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
import org.sciborgs1155.lib.Vision;
import org.sciborgs1155.lib.constants.PIDConfigurer;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.SwerveModule.Driving;
import org.sciborgs1155.robot.Constants.SwerveModule.Turning;
import org.sciborgs1155.robot.subsystems.modules.SwerveModule;

public class Drive extends SubsystemBase implements Loggable {

  @Log
  private final SwerveModule frontLeft =
      SwerveModule.create(FRONT_LEFT_DRIVE, FRONT_LEFT_TURNING, ANGULAR_OFFSETS[0]);

  @Log
  private final SwerveModule frontRight =
      SwerveModule.create(FRONT_RIGHT_DRIVE, FRONT_RIGHT_TURNING, ANGULAR_OFFSETS[1]);

  @Log
  private final SwerveModule rearLeft =
      SwerveModule.create(REAR_LEFT_DRIVE, REAR_LEFT_TURNING, ANGULAR_OFFSETS[2]);

  @Log
  private final SwerveModule rearRight =
      SwerveModule.create(REAR_RIGHT_DRIVE, REAR_RIGHT_TURNING, ANGULAR_OFFSETS[3]);

  private final SwerveModule[] modules = {frontLeft, frontRight, rearLeft, rearRight};

  // PID configurations for swerve modules
  @Log private final PIDConfigurer moduleDrivePID = new PIDConfigurer(Driving.PID);
  @Log private final PIDConfigurer moduleTurnPID = new PIDConfigurer(Turning.PID);

  @Log private final WPI_PigeonIMU imu = new WPI_PigeonIMU(PIGEON);

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(MODULE_OFFSET);

  // Odometry and pose estimation
  private final Vision vision;
  private final SwerveDrivePoseEstimator odometry =
      new SwerveDrivePoseEstimator(kinematics, getHeading(), getModulePositions(), new Pose2d());

  @Log private final Field2d field2d = new Field2d();
  private final FieldObject2d[] modules2d = new FieldObject2d[modules.length];

  // Rate limiting
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(MAX_ACCEL);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(MAX_ACCEL);

  public Drive(Vision vision) {
    this.vision = vision;

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
    return odometry.getEstimatedPosition();
  }

  /**
   * Returns the heading of the robot, based on our pigeon
   *
   * @return A Rotation2d of our angle
   */
  public Rotation2d getHeading() {
    return imu.getRotation2d();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getHeading(), getModulePositions(), pose);
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
    xSpeed = xLimiter.calculate(Math.pow(xSpeed, 2) * Math.signum(xSpeed) * MAX_SPEED);
    ySpeed = yLimiter.calculate(Math.pow(ySpeed, 2) * Math.signum(ySpeed) * MAX_SPEED);
    rot = Math.pow(rot, 2) * Math.signum(rot) * MAX_ANGULAR_SPEED;

    var speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);

    if (fieldRelative) {
      speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              speeds, odometry.getEstimatedPosition().getRotation());
    }

    setModuleStates(kinematics.toSwerveModuleStates(speeds));
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

    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_SPEED);

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
    imu.reset();
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
    return imu.getRate();
  }

  /**
   * Returns the pitch value recorded by the pigeon.
   *
   * @return The pitch value of the pigeon.
   */
  @Log
  public double getPitch() {
    // TODO make this account for pitch and roll
    return imu.getPitch();
  }

  @Override
  public void periodic() {
    odometry.update(getHeading(), getModulePositions());

    var poses = vision.getPoseEstimates(getPose());

    for (int i = 0; i < poses.length; i++) {
      odometry.addVisionMeasurement(poses[i].estimatedPose.toPose2d(), poses[i].timestampSeconds);
      field2d.getObject("Cam-" + i + " Est Pose").setPose(poses[i].estimatedPose.toPose2d());
    }

    field2d.setRobotPose(getPose());

    for (int i = 0; i < modules2d.length; i++) {
      var transform = new Transform2d(MODULE_OFFSET[i], modules[i].getPosition().angle);
      modules2d[i].setPose(getPose().transformBy(transform));
    }

    for (var module : modules) {
      module.setDrivePID(moduleDrivePID.get());
      module.setTurnPID(moduleTurnPID.get());
    }
  }

  @Override
  public void simulationPeriodic() {
    imu.getSimCollection()
        .addHeading(
            Units.radiansToDegrees(
                    kinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond)
                * Constants.RATE);

    vision.updateSeenTags();
  }

  /**
   * Follows a path on the field.
   *
   * @param trajectory The pathplanner trajectory the robot will follow
   * @param resetPosition Whether the robot should set its odometry to the initial pose of the
   *     trajectory
   * @param useAllianceColor Whether the robot should take into account alliance color before
   *     following
   * @return The command that follows the trajectory
   */
  public Command follow(
      PathPlannerTrajectory trajectory, boolean resetPosition, boolean useAllianceColor) {
    if (resetPosition) resetOdometry(trajectory.getInitialPose());

    return new PPSwerveControllerCommand(
            trajectory,
            this::getPose,
            kinematics,
            CARTESIAN.create(),
            CARTESIAN.create(),
            ANGULAR.create(),
            this::setModuleStates,
            useAllianceColor)
        .andThen(stop());
  }

  /** Follows the specified path planner path given a path name */
  public Command follow(String pathName, boolean resetPosition, boolean useAllianceColor) {
    PathPlannerTrajectory loadedPath = PathPlanner.loadPath(pathName, CONSTRAINTS);
    return follow(loadedPath, resetPosition, useAllianceColor);
  }

  /** Drives robot based on three double suppliers (x,y and rot) */
  public Command drive(
      DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot, boolean fieldRelative) {
    return run(
        () ->
            drive(
                MathUtil.applyDeadband(x.getAsDouble(), Constants.DEADBAND),
                MathUtil.applyDeadband(y.getAsDouble(), Constants.DEADBAND),
                MathUtil.applyDeadband(rot.getAsDouble(), Constants.DEADBAND),
                fieldRelative));
  }
  /** Stops drivetrain */
  public Command stop() {
    return run(() -> setModuleStates(getModuleStates()));
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
    return run(() -> setModuleStates(states));
  }
}
