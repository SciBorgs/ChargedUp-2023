package org.sciborgs1155.lib;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.io.IOException;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.Dimensions;

public class VisionWrapper {
  /** Dimensions wrapper for mechanisms */

  // Pose Estimation & Alignment
  public final PhotonCamera frontCam = new PhotonCamera(Constants.FRONT_CAM);
  ;

  public final PhotonCamera backCam = new PhotonCamera(Constants.BACK_CAM);
  ;
  public AprilTagFieldLayout tagLayout;

  /** Creates a new Dimensions. */
  public VisionWrapper() {
    try {
      tagLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  public boolean hasTargets() {
    return frontCam.getLatestResult().hasTargets() && backCam.getLatestResult().hasTargets();
  }

  /* POSE ESTIMATION */
  private double calculateDifference(Pose3d x, Pose3d y) {
    return x.getTranslation().getDistance(y.getTranslation());
  }

  /* Gets the true best target from both camera's inputs USING CLOSEST_REFERENCE_POSE strategy */
  private EstimatedRobotPose getBestReferenceTarget(
      EstimatedRobotPose frontVisionPose, EstimatedRobotPose backVisionPose, Pose3d referencePose) {

    double camOneDiff = Math.abs(calculateDifference(frontVisionPose.estimatedPose, referencePose));
    double camTwoDiff = Math.abs(calculateDifference(backVisionPose.estimatedPose, referencePose));
    return (camOneDiff < camTwoDiff) ? frontVisionPose : backVisionPose;
  }
  /* Gets the true best target from both camera's inputs USING LOWEST_AMBIGUITY strategy */
  private EstimatedRobotPose getLowestAmbiguityTarget(
      PhotonPipelineResult frontCamResult, PhotonPipelineResult backCamResult) {

    PhotonTrackedTarget frontCamBestTarget = frontCamResult.getBestTarget();
    PhotonTrackedTarget backCamBestTarget = backCamResult.getBestTarget();
    int fiducialId;
    Pose3d bestTagPose;

    if (frontCamBestTarget.getPoseAmbiguity() < backCamBestTarget.getPoseAmbiguity()) {
      fiducialId = frontCamBestTarget.getFiducialId();
      bestTagPose = tagLayout.getTagPose(fiducialId).get();
      return new EstimatedRobotPose(
          bestTagPose
              .transformBy(frontCamBestTarget.getBestCameraToTarget().inverse())
              .transformBy(Dimensions.ROBOT_TO_FRONT_CAM.inverse()),
          frontCamResult.getTimestampSeconds(),
          backCamResult.getTargets());
    } else {
      fiducialId = backCamBestTarget.getFiducialId();
      bestTagPose = tagLayout.getTagPose(fiducialId).get();
      return new EstimatedRobotPose(
          bestTagPose
              .transformBy(backCamBestTarget.getBestCameraToTarget().inverse())
              .transformBy(Dimensions.ROBOT_TO_BACK_CAM.inverse()),
          backCamResult.getTimestampSeconds(),
          backCamResult.getTargets());
    }
  }

  /* Gets estimated pose from vision measurements */
  private Optional<EstimatedRobotPose> getVisionEstimate(
      PhotonPoseEstimator visionOdometry, SwerveDrivePoseEstimator odometry) {
    visionOdometry.setReferencePose(odometry.getEstimatedPosition());
    visionOdometry.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    Optional<EstimatedRobotPose> visionEstimate = visionOdometry.update();
    return visionEstimate;
  }

  public void updateVisionOdometry(
      PhotonPoseEstimator frontVisionOdometry,
      PhotonPoseEstimator backVisionOdometry,
      SwerveDrivePoseEstimator driveOdometry,
      Field2d field) {

    EstimatedRobotPose frontVisionPose;
    EstimatedRobotPose backVisionPose;
    Pose2d bestTargetPose;
    EstimatedRobotPose bestPoseEstimate;
    Pose3d backPoseTransform;
    EstimatedRobotPose transformedBackVisionPose;

    if (hasTargets()) {
      frontVisionPose = getVisionEstimate(frontVisionOdometry, driveOdometry).get();
      backVisionPose = getVisionEstimate(backVisionOdometry, driveOdometry).get();
      backPoseTransform = backVisionPose.estimatedPose.transformBy(Dimensions.ROBOT_TO_BACK_CAM);
      transformedBackVisionPose =
          new EstimatedRobotPose(
              backPoseTransform, backVisionPose.timestampSeconds, backVisionPose.targetsUsed);

      switch (Dimensions.SECONDARY_POSE_STRATEGY) {
        case CLOSEST_TO_REFERENCE_POSE:
          {
            bestPoseEstimate =
                getBestReferenceTarget(
                    frontVisionPose,
                    transformedBackVisionPose,
                    new Pose3d(driveOdometry.getEstimatedPosition()));
            break;
          }
        case LOWEST_AMBIGUITY:
          {
            bestPoseEstimate =
                getLowestAmbiguityTarget(frontCam.getLatestResult(), backCam.getLatestResult());
            break;
          }
        default:
          throw new UnsupportedOperationException(
              "Check the enum; only CLOSEST_TO_REFERENCE_POSE and LOWEST_AMBIGUITY can be used");
      }
      bestTargetPose = bestPoseEstimate.estimatedPose.toPose2d();
      // System.out.println("Best Pose: " + bestTargetPose);
      driveOdometry.addVisionMeasurement(bestTargetPose, bestPoseEstimate.timestampSeconds);
      field.getObject("Cam Est Pose").setPose(bestTargetPose);
    } else {
      field.getObject("Cam Est Pose").setPose(new Pose2d(-100, -100, new Rotation2d()));
    }
  }
}
