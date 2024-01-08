package org.sciborgs1155.robot.vision;

import static org.sciborgs1155.robot.vision.VisionConstants.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.stream.Stream;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Wrapper class for photonvision */
public class RealVision implements VisionIO {

  private final PhotonCamera frontCam = new PhotonCamera(FRONT_CAM);
  private final PhotonCamera backCam = new PhotonCamera(BACK_CAM);

  private final PhotonPoseEstimator frontEstimator;
  private final PhotonPoseEstimator backEstimator;

  private AprilTagFieldLayout layout;

  public RealVision() {
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

    frontEstimator.setMultiTagFallbackStrategy(SECONDARY_POSE_STRATEGY);
    backEstimator.setMultiTagFallbackStrategy(SECONDARY_POSE_STRATEGY);
  }

  public AprilTagFieldLayout getLayout() {
    return layout;
  }

  public boolean hasTargets() {
    return frontCam.getLatestResult().hasTargets() || backCam.getLatestResult().hasTargets();
  }

  /** Use hasTargets() before calling */
  // note that getBestTarget() isn't updated to support apriltags
  public PhotonTrackedTarget getBestTarget() {
    return Stream.of(frontCam, backCam)
        .map(PhotonCamera::getLatestResult)
        .map(PhotonPipelineResult::getBestTarget)
        .toList()
        .stream()
        .sorted(Comparator.comparingDouble(PhotonTrackedTarget::getPoseAmbiguity))
        .toArray(PhotonTrackedTarget[]::new)[0];
  }

  /* Gets estimated pose from vision measurements */
  public EstimatedRobotPose[] getPoseEstimates(Pose2d lastPose) {
    frontEstimator.setReferencePose(lastPose);
    backEstimator.setReferencePose(lastPose);

    return Stream.of(frontEstimator, backEstimator)
        .map(PhotonPoseEstimator::update)
        .flatMap(Optional::stream)
        .toArray(EstimatedRobotPose[]::new);
  }

  private List<PhotonTrackedTarget> filterTargets(PhotonCamera cam) {
    return filterTargets(cam.getLatestResult().getTargets());
  }

  /** Filter out tags that are too ambiguous */
  private List<PhotonTrackedTarget> filterTargets(List<PhotonTrackedTarget> targets) {
    double inaccThreshold = 0.12;
    for (PhotonTrackedTarget target : targets) {
      if (target.getPoseAmbiguity() > inaccThreshold) {
        targets.remove(target);
      }
    }
    return targets;
  }

  public void updateSeenTags() {
    SmartDashboard.putNumberArray(
        "visible front tags", createObjects(determineSeenTags(frontCam, layout)));
    SmartDashboard.putNumberArray(
        "visible back tags", createObjects(determineSeenTags(backCam, layout)));
  }

  /** Determines which tags are in view, if any (for advantagescope) */
  private Pose3d[] determineSeenTags(PhotonCamera cam, AprilTagFieldLayout layout) {
    return cam.getLatestResult().getTargets().stream()
        .filter(target -> target.getFiducialId() != -1)
        .map(PhotonTrackedTarget::getFiducialId)
        .map(layout::getTagPose)
        .map(Optional::get)
        .toArray(Pose3d[]::new);
  }

  private double[] createObjects(Pose3d[] poses) {
    double[] data = new double[poses.length * 7];
    for (int i = 0; i < poses.length; i++) {
      data[i * 7] = poses[i].getX();
      data[i * 7 + 1] = poses[i].getY();
      data[i * 7 + 2] = poses[i].getZ();
      data[i * 7 + 3] = poses[i].getRotation().getQuaternion().getW();
      data[i * 7 + 4] = poses[i].getRotation().getQuaternion().getX();
      data[i * 7 + 5] = poses[i].getRotation().getQuaternion().getY();
      data[i * 7 + 6] = poses[i].getRotation().getQuaternion().getZ();
    }
    return data;
  }

  @Override
  public void close() {
    frontCam.close();
    backCam.close();
  }
}
