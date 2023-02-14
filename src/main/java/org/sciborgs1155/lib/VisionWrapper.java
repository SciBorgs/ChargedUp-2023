// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.lib;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.Vision;

public class VisionWrapper {
  /** Vision wrapper for mechanisms */

  // Pose Estimation & Alignment
  public final PhotonCamera frontCam;

  public final PhotonCamera backCam;
  public final AprilTagFieldLayout tagLayout;

  /** Creates a new Vision. */
  public VisionWrapper() {
    frontCam = new PhotonCamera(Vision.FRONT_CAMERA);
    backCam = new PhotonCamera(Vision.BACK_CAMERA);
    tagLayout =
        new AprilTagFieldLayout(
            Constants.Vision.AprilTagPose.APRIL_TAGS,
            Constants.Vision.FIELD_LENGTH,
            Constants.Vision.FIELD_WIDTH);
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
          bestTagPose.transformBy(frontCamBestTarget.getBestCameraToTarget().inverse()),
          // .transformBy(Vision.ROBOT_TO_CAM.inverse()),
          frontCamResult.getTimestampSeconds());
    } else {
      fiducialId = backCamBestTarget.getFiducialId();
      bestTagPose = tagLayout.getTagPose(fiducialId).get();
      return new EstimatedRobotPose(
          bestTagPose.transformBy(backCamBestTarget.getBestCameraToTarget().inverse()),
          // .transformBy(Vision.ROBOT_TO_CAM.inverse()),
          backCamResult.getTimestampSeconds());
    }
  }

  /* Gets estimated pose from vision measurements */
  private Optional<EstimatedRobotPose> getVisionEstimate(
      PhotonPoseEstimator visionOdometry, SwerveDrivePoseEstimator odometry) {
    visionOdometry.setReferencePose(odometry.getEstimatedPosition());
    Optional<EstimatedRobotPose> visionEstimate = visionOdometry.update();
    return visionEstimate;
  }

  /* Updates SwerveDrivePoseEstimator with vision measurements based on pose strategy */
  public void updateVisionOdometry(
      PhotonPoseEstimator frontVisionOdometry,
      PhotonPoseEstimator backVisionOdometry,
      SwerveDrivePoseEstimator driveOdometry,
      Field2d field) {

    EstimatedRobotPose frontVisionPose;
    EstimatedRobotPose backVisionPose;
    Pose2d bestTargetPose;

    EstimatedRobotPose bestPoseEstimate = new EstimatedRobotPose(new Pose3d(), 0);

    if (hasTargets()) {
      frontVisionPose = getVisionEstimate(frontVisionOdometry, driveOdometry).get();
      backVisionPose = getVisionEstimate(backVisionOdometry, driveOdometry).get();
      switch (Vision.SECONDARY_POSE_STRATEGY) {
        case "CLOSEST_REFERENCE_POSE":
          {
            bestPoseEstimate =
                getBestReferenceTarget(
                    frontVisionPose,
                    backVisionPose,
                    new Pose3d(driveOdometry.getEstimatedPosition()));
            break;
          }
        case "LOWEST_AMBIGUITY":
          {
            bestPoseEstimate =
                getLowestAmbiguityTarget(frontCam.getLatestResult(), backCam.getLatestResult());
            break;
          }
      }
      bestTargetPose = bestPoseEstimate.estimatedPose.toPose2d();
      driveOdometry.addVisionMeasurement(bestTargetPose, bestPoseEstimate.timestampSeconds);
      field.getObject("Cam Est Pose").setPose(bestTargetPose);
    } else {
      field.getObject("Cam Est Pose").setPose(new Pose2d(-100, -100, new Rotation2d()));
    }
  }
}
