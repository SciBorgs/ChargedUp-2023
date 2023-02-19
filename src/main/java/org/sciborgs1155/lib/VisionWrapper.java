package org.sciborgs1155.lib;

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
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.SimVisionSystem;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.Dimensions;
import org.sciborgs1155.robot.Constants.Dimensions.VisionSim;
import org.sciborgs1155.robot.Robot;

public class VisionWrapper {
  /** Dimensions wrapper for mechanisms */

  // Pose Estimation & Alignment
  public final PhotonCamera frontCam;

  public final PhotonCamera backCam;

  public final PhotonPoseEstimator frontVisionOdometry;
  public final PhotonPoseEstimator backVisionOdometry;

  public final SimVisionSystem simFront;
  public final SimVisionSystem simBack;

  /** Creates a new Constants. */
  public VisionWrapper() {
    frontCam = new PhotonCamera(Constants.FRONT_CAM);
    backCam = new PhotonCamera(Constants.BACK_CAM);

    AprilTagFieldLayout layout;
    try {
      layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
    } catch (Exception e) {
      layout = new AprilTagFieldLayout(new ArrayList<AprilTag>(), 0, 0);
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
    }
    frontVisionOdometry =
        new PhotonPoseEstimator(
            layout, Constants.PRIMARY_POSE_STRATEGY, frontCam, Dimensions.ROBOT_TO_FRONT_CAM);
    backVisionOdometry =
        new PhotonPoseEstimator(
            layout, Constants.PRIMARY_POSE_STRATEGY, backCam, Dimensions.ROBOT_TO_BACK_CAM);
    simFront =
        new SimVisionSystem(
            Constants.FRONT_CAM,
            VisionSim.camDiagFOVDegrees,
            Dimensions.ROBOT_TO_FRONT_CAM,
            VisionSim.maxLEDRangeMeters,
            VisionSim.CAMERA_RES_WIDTH,
            VisionSim.CAMERA_RES_HEIGHT,
            VisionSim.minTargetArea);
    simBack =
        new SimVisionSystem(
            Constants.BACK_CAM,
            VisionSim.camDiagFOVDegrees,
            Dimensions.ROBOT_TO_BACK_CAM,
            VisionSim.maxLEDRangeMeters,
            VisionSim.CAMERA_RES_WIDTH,
            VisionSim.CAMERA_RES_HEIGHT,
            VisionSim.minTargetArea);

    simFront.addVisionTargets(layout);
    simBack.addVisionTargets(layout);

    frontVisionOdometry.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    backVisionOdometry.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  /* Gets estimated pose from vision measurements */
  public EstimatedRobotPose[] getPoseEstimates(Pose2d lastPose) {
    frontVisionOdometry.setReferencePose(lastPose);
    backVisionOdometry.setReferencePose(lastPose);
    if (Robot.isSimulation()) {
      simFront.processFrame(lastPose);
      simBack.processFrame(lastPose);
    }
    return Stream.of(frontVisionOdometry, backVisionOdometry)
        .map(PhotonPoseEstimator::update)
        .filter(Optional::isPresent)
        .map(Optional::get)
        .toArray(EstimatedRobotPose[]::new);
  }
}
