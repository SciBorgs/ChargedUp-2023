package org.sciborgs1155.robot.subsystems;

import static org.sciborgs1155.robot.Constants.Drive.*;
import static org.sciborgs1155.robot.Ports.Drive.*;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import java.util.Arrays;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.sciborgs1155.lib.Camera;
import org.photonvision.SimVisionSystem;
import org.sciborgs1155.lib.ControllerOutputFunction;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.Auto;
import org.sciborgs1155.robot.Constants.Vision;
import org.sciborgs1155.robot.Constants.Vision.VisionSim;
import org.sciborgs1155.robot.Ports.Sensors;
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

  // The gyro sensor
  @Log private final WPI_PigeonIMU gyro = new WPI_PigeonIMU(Sensors.PIGEON);
  private final ADIS16470_IMU imu = new ADIS16470_IMU();

  // Odometry and pose estimation
  private final SwerveDrivePoseEstimator odometry;
  private final AprilTagFieldLayout layout;
  private final PhotonCamera cam;
  private final PhotonPoseEstimator visionOdometry;
  private final SimVisionSystem sim;
  @Log private final Field2d field2d = new Field2d();

  private final FieldObject2d[] modules2d = new FieldObject2d[modules.length];

  public Drive(PhotonCamera cam) {
    this.cam = cam;   
    layout =
        new AprilTagFieldLayout(
            Vision.AprilTagPose.APRIL_TAGS, Vision.FIELD_LENGTH, Vision.FIELD_WIDTH);
    odometry =
        new SwerveDrivePoseEstimator(
            KINEMATICS,
            getHeading(),
            getModulePositions(),
            new Pose2d()); // TODO change to initial pose
    visionOdometry =
        new PhotonPoseEstimator(layout, PoseStrategy.LOWEST_AMBIGUITY, cam, Vision.ROBOT_TO_CAM);
    sim =
        new SimVisionSystem(
            Vision.CAM_NAME,
            VisionSim.camDiagFOVDegrees,
            Vision.ROBOT_TO_CAM,
            VisionSim.maxLEDRangeMeters,
            VisionSim.CAMERA_RES_WIDTH,
            VisionSim.CAMERA_RES_HEIGHT,
            VisionSim.minTargetArea);
    for (int i = 0; i < modules2d.length; i++) modules2d[i] = field2d.getObject("module-" + i);

    sim.addVisionTargets(layout);
    SmartDashboard.putData("Field", field2d);
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
   * Returns the heading of the robot, based on our imu
   *
   * <p>The imu is ccw positive, but mounted upside down
   *
   * @return A Rotation2d of our angle
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(imu.getAngle());
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
    // scale inputs based on maximum values
    xSpeed *= MAX_SPEED;
    ySpeed *= MAX_SPEED;
    rot *= MAX_ANGULAR_SPEED;

    var states =
        KINEMATICS.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading())
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
    return gyro.getRate() * (GYRO_REVERSED ? -1.0 : 1.0);
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

  private void updateOdometry() {
    // Real field odometry
    odometry.update(getHeading(), getModulePositions());

    var latest = cam.getLatestResult();
    var bestTarget = latest.getBestTarget();
    visionOdometry.setReferencePose(odometry.getEstimatedPosition());
    Optional<EstimatedRobotPose> visionEstimate = visionOdometry.update();

    if (latest.hasTargets()) {
      EstimatedRobotPose visionPose = visionEstimate.get();
      Pose2d visionPoseEstimate = visionPose.estimatedPose.toPose2d();
      odometry.addVisionMeasurement(visionPoseEstimate, visionPose.timestampSeconds);
      
      // sim
      field2d.getObject("Cam Est Pose").setPose(visionPoseEstimate);
    } else {
      field2d.getObject("Cam Est Pose").setPose(new Pose2d(-100, -100, new Rotation2d()));
    }
    field2d.getObject("Actual Pose").setPose(getPose());
    field2d.setRobotPose(getPose());
    sim.processFrame(getPose());
  }

  @Override
  public void periodic() {
    updateOdometry();
    field2d.setRobotPose(getPose());
    for (int i = 0; i < modules2d.length; i++) {
      var transform = new Transform2d(MODULE_OFFSET[i], modules[i].getPosition().angle);
      modules2d[i].setPose(getPose().transformBy(transform));
    }
  }

  @Override
  public void simulationPeriodic() {
    // ArrayList<PhotonTrackedTarget> visibleTgtList = new ArrayList<PhotonTrackedTarget>();
    // for (AprilTag aprilTag : Vision.AprilTagPose.APRIL_TAGS) {
    //   visibleTgtList.add(new PhotonTrackedTarget(aprilTag.pose.getX(),
    //   aprilTag.pose.getRotation().getAngle(),
    //   ));
    // }

    gyro.getSimCollection()
        .addHeading(
            Units.radiansToDegrees(
                KINEMATICS.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond
                    * Constants.RATE));
  }

  public Command follow(PathPlannerTrajectory trajectory) {
    PIDController x = new PIDController(Auto.Cartesian.kP, Auto.Cartesian.kI, Auto.Cartesian.kD);
    PIDController y = new PIDController(Auto.Cartesian.kP, Auto.Cartesian.kI, Auto.Cartesian.kD);
    PIDController rot = new PIDController(Auto.Angular.kP, Auto.Angular.kI, Auto.Angular.kD);

    resetOdometry(trajectory.getInitialPose());
    return new PPSwerveControllerCommand(
            trajectory, this::getPose, KINEMATICS, x, y, rot, this::setModuleStates, false)
        .andThen(stop());
  }

  /** Follows the specified path planner path */
  public Command follow(String pathName) {
    PathPlannerTrajectory loadedPath = PathPlanner.loadPath(pathName, Auto.CONSTRAINTS);
    return follow(loadedPath);
  }

  /** Drive based on xbox */
  public Command drive(CommandXboxController xbox, boolean fieldRelative) {
    return run(
        () -> {
          double rawX = -xbox.getLeftY();
          double rawY = -xbox.getLeftX();
          double rawSpeed = Math.sqrt(rawX * rawX + rawY * rawY);
          double speedFactor = mapper.map(rawSpeed) / rawSpeed;
          double rawOmega = -xbox.getRightX();
          drive(
              MathUtil.applyDeadband(rawX * speedFactor, Constants.DEADBAND),
              MathUtil.applyDeadband(rawY * speedFactor, Constants.DEADBAND),
              MathUtil.applyDeadband(mapper.map(rawOmega), Constants.DEADBAND),
              fieldRelative);
        });
  }

  // TODO replace
  private static final ControllerOutputFunction mapper =
      ControllerOutputFunction.powerExp(Math.E, Math.PI);

  /** Drive based on joysticks */
  public Command drive(CommandJoystick left, CommandJoystick right, boolean fieldRelative) {
    return run(
        () -> {
          double rawX = -left.getY();
          double rawY = -left.getX();
          double rawSpeed = Math.sqrt(rawX * rawX + rawY * rawY);
          double speedFactor = mapper.map(rawSpeed) / rawSpeed;
          double rawOmega = -right.getX();
          drive(
              MathUtil.applyDeadband(speedFactor * rawX, Constants.DEADBAND),
              MathUtil.applyDeadband(speedFactor * rawY, Constants.DEADBAND),
              MathUtil.applyDeadband(mapper.map(rawOmega), Constants.DEADBAND),
              fieldRelative);
        });
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
