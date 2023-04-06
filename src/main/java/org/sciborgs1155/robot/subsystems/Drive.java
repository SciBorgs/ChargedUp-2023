package org.sciborgs1155.robot.subsystems;

import static org.sciborgs1155.robot.Constants.Drive.*;
import static org.sciborgs1155.robot.Ports.Drive.*;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.subsystems.modules.SwerveModule;
import org.sciborgs1155.robot.util.Vision;

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

  private final List<SwerveModule> modules = List.of(frontLeft, frontRight, rearLeft, rearRight);

  @Log private final WPI_PigeonIMU imu = new WPI_PigeonIMU(PIGEON);

  public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(MODULE_OFFSET);

  // Odometry and pose estimation
  private final Vision vision;
  private final SwerveDrivePoseEstimator odometry =
      new SwerveDrivePoseEstimator(kinematics, getHeading(), getModulePositions(), new Pose2d());

  @Log private final Field2d field2d = new Field2d();
  private final FieldObject2d[] modules2d = new FieldObject2d[modules.size()];

  // Rate limiting
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(MAX_ACCEL);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(MAX_ACCEL);

  @Log private double speedMultiplier = SpeedMultiplier.NORMAL.multiplier;

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
    // tmp
    speedMultiplier = 1;
    xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed) * MAX_SPEED * speedMultiplier;
    ySpeed = Math.copySign(ySpeed * ySpeed, ySpeed) * MAX_SPEED * speedMultiplier;
    rot = Math.copySign(rot * rot, rot) * MAX_ANGULAR_SPEED * speedMultiplier;

    if (fieldRelative) {
      xSpeed = xLimiter.calculate(xSpeed);
      ySpeed = yLimiter.calculate(ySpeed);
    }

    var speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);

    setSpeeds(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading()) : speeds);
  }

  /** Sets the swerve ModuleStates via a {@link ChassisSpeeds} */
  public void setSpeeds(ChassisSpeeds speeds) {
    setModuleStates(kinematics.toSwerveModuleStates(speeds));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    if (desiredStates.length != modules.size()) {
      throw new IllegalArgumentException("desiredStates must have the same length as modules");
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_SPEED);

    for (int i = 0; i < modules.size(); i++) {
      modules.get(i).setDesiredState(desiredStates[i]);
    }
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    modules.forEach(SwerveModule::resetEncoders);
  }

  /** Zeroes the heading of the robot. */
  public Command zeroHeading() {
    return runOnce(imu::reset);
  }

  private SwerveModuleState[] getModuleStates() {
    return modules.stream().map(SwerveModule::getState).toArray(SwerveModuleState[]::new);
  }

  private SwerveModulePosition[] getModulePositions() {
    return modules.stream().map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new);
  }

  /**
   * Returns the pitch value recorded by the pigeon.
   *
   * @return The pitch value of the pigeon.
   */
  @Log
  public double getPitch() {
    return imu.getPitch();
  }

  /**
   * Returns the roll value recorded by the pigeon.
   *
   * @return The roll value of the pigeon.
   */
  @Log
  public double getRoll() {
    return imu.getRoll();
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
      var transform = new Transform2d(MODULE_OFFSET[i], modules.get(i).getPosition().angle);
      modules2d[i].setPose(getPose().transformBy(transform));
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

  /** Sets a new speed multiplier for the robot, this affects max cartesian and angular speeds */
  public Command setSpeedMultiplier(SpeedMultiplier multiplier) {
    return runOnce(() -> speedMultiplier = multiplier.multiplier);
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
            new PIDController(TRANSLATION.p(), TRANSLATION.i(), TRANSLATION.d()),
            new PIDController(TRANSLATION.p(), TRANSLATION.i(), TRANSLATION.d()),
            new PIDController(ROTATION.p(), ROTATION.i(), ROTATION.d()),
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

  public Command balance() {
    DoubleUnaryOperator velocity =
        pitch -> Math.signum(MathUtil.applyDeadband(pitch, MIN_PITCH)) * BALANCE_SPEED;

    return run(() -> setSpeeds(new ChassisSpeeds(velocity.applyAsDouble(getPitch()), 0, 0)))
        .until(() -> Math.abs(getPitch()) < MIN_PITCH)
        .andThen(stop());
  }

  /** Stops drivetrain */
  public Command stop() {
    return drive(() -> 0, () -> 0, () -> 0, false);
  }

  /** Sets the drivetrain to an "X" configuration, preventing movement */
  public Command lock() {
    var front = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
    var back = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
    return run(() -> setModuleStates(new SwerveModuleState[] {front, back, back, front}));
  }

  /** Creates and follows trajectroy for swerve from startPose to desiredPose */
  public Command driveToPose(Pose2d startPose, Pose2d desiredPose, boolean useAllianceColor) {
    Rotation2d heading = desiredPose.minus(startPose).getTranslation().getAngle();
    PathPoint start = new PathPoint(startPose.getTranslation(), heading, startPose.getRotation());
    PathPoint goal =
        new PathPoint(desiredPose.getTranslation(), heading, desiredPose.getRotation());
    PathPlannerTrajectory trajectory = PathPlanner.generatePath(CONSTRAINTS, start, goal);
    return follow(trajectory, false, useAllianceColor);
  }

  /** Creates and follows trajectory for swerve from current pose to desiredPose */
  public Command driveToPose(Pose2d desiredPose, boolean useAllianceColor) {
    return driveToPose(getPose(), desiredPose, useAllianceColor);
  }
}
