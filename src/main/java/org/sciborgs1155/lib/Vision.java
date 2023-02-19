package org.sciborgs1155.lib;

import static org.sciborgs1155.robot.Constants.Vision.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
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
  }

  /* Gets estimated pose from vision measurements */
  public EstimatedRobotPose[] getPoseEstimates(Pose2d lastPose) {
    frontEstimator.setReferencePose(lastPose);
    backEstimator.setReferencePose(lastPose);

    if (Robot.isSimulation()) {
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
