package org.sciborgs1155.lib;

import static org.sciborgs1155.robot.Constants.Vision.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.Optional;
import java.util.stream.Stream;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.SimVisionSystem;
import org.sciborgs1155.robot.Robot;

public class Vision {

  public enum Mode {
    REAL,
    SIM,
    SIM_WITH_CAM;
  }

  private final Mode mode;

  private final PhotonCamera frontCam = new PhotonCamera(FRONT_CAM);
  private final PhotonCamera backCam = new PhotonCamera(BACK_CAM);

  private final PhotonPoseEstimator frontEstimator;
  private final PhotonPoseEstimator backEstimator;

  private final SimVisionSystem simFront =
      new SimVisionSystem(
          FRONT_CAM,
          CAM_FOV,
          ROBOT_TO_FRONT_CAM,
          LED_RANGE,
          CAMERA_RES_WIDTH,
          CAMERA_RES_HEIGHT,
          MIN_TARGET_AREA);
  private final SimVisionSystem simBack =
      new SimVisionSystem(
          BACK_CAM,
          CAM_FOV,
          ROBOT_TO_BACK_CAM,
          LED_RANGE,
          CAMERA_RES_WIDTH,
          CAMERA_RES_HEIGHT,
          MIN_TARGET_AREA);

  public Vision() {
    this(Robot.isReal() ? Mode.REAL : Mode.SIM);
  }

  public Vision(Mode mode) {
    AprilTagFieldLayout layout;
    try {
      layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
    } catch (Exception e) {
      layout = new AprilTagFieldLayout(new ArrayList<AprilTag>(), 0, 0);
      DriverStation.reportError(
          "Failed to load AprilTagFieldLayout, odometry will not work correctly",
          e.getStackTrace());
    }

    frontEstimator =
        new PhotonPoseEstimator(layout, PRIMARY_POSE_STRATEGY, frontCam, ROBOT_TO_FRONT_CAM);
    backEstimator =
        new PhotonPoseEstimator(layout, PRIMARY_POSE_STRATEGY, backCam, ROBOT_TO_BACK_CAM);

    simFront.addVisionTargets(layout);
    simBack.addVisionTargets(layout);

    frontEstimator.setMultiTagFallbackStrategy(SECONDARY_POSE_STRATEGY);
    backEstimator.setMultiTagFallbackStrategy(SECONDARY_POSE_STRATEGY);

    this.mode = mode;

    // set up networktables if we are in hardware sim mode
    if (mode == Mode.SIM_WITH_CAM) {
      if (Robot.isReal()) {
        throw new IllegalStateException(
            "The robot is real and cannot be used in mode SIM_WITH_CAM");
      }
      var inst = NetworkTableInstance.getDefault();
      inst.stopServer();
      inst.setServer("photonvision.local");
      inst.startClient4("Robot Simulation");
    }
  }

  /* Gets estimated pose from vision measurements */
  public EstimatedRobotPose[] getPoseEstimates(Pose2d lastPose) {
    frontEstimator.setReferencePose(lastPose);
    backEstimator.setReferencePose(lastPose);

    if (mode == Mode.SIM) {
      simFront.processFrame(lastPose);
      simBack.processFrame(lastPose);
    }

    return Stream.of(frontEstimator, backEstimator)
        .map(PhotonPoseEstimator::update)
        .filter(Optional::isPresent)
        .map(Optional::get)
        .toArray(EstimatedRobotPose[]::new);
  }
}
