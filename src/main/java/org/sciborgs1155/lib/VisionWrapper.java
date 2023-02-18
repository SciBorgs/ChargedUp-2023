// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.lib;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.Optional;
import java.util.stream.Stream;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.SimVisionSystem;
import org.sciborgs1155.robot.Constants.Vision;
import org.sciborgs1155.robot.Constants.Vision.VisionSim;
import org.sciborgs1155.robot.Robot;

public class VisionWrapper {
  /** Vision wrapper for mechanisms */

  // Pose Estimation & Alignment
  public final PhotonCamera frontCam;

  public final PhotonCamera backCam;

  public final AprilTagFieldLayout tagLayout;

  public final PhotonPoseEstimator frontVisionOdometry;
  public final PhotonPoseEstimator backVisionOdometry;

  public final SimVisionSystem simFront;
  public final SimVisionSystem simBack;

  /** Creates a new Vision. */
  public VisionWrapper() {
    frontCam = new PhotonCamera(Vision.FRONT_CAMERA);
    backCam = new PhotonCamera(Vision.BACK_CAMERA);
    tagLayout =
        new AprilTagFieldLayout(
            Vision.AprilTagPose.APRIL_TAGS, Vision.FIELD_LENGTH, Vision.FIELD_WIDTH);
    frontVisionOdometry =
        new PhotonPoseEstimator(
            tagLayout, Vision.PRIMARY_POSE_STRATEGY, frontCam, Vision.ROBOT_TO_FRONT_CAM);
    backVisionOdometry =
        new PhotonPoseEstimator(
            tagLayout, Vision.PRIMARY_POSE_STRATEGY, backCam, Vision.ROBOT_TO_BACK_CAM);
    simFront =
        new SimVisionSystem(
            Vision.FRONT_CAMERA,
            VisionSim.camDiagFOVDegrees,
            Vision.ROBOT_TO_FRONT_CAM,
            VisionSim.maxLEDRangeMeters,
            VisionSim.CAMERA_RES_WIDTH,
            VisionSim.CAMERA_RES_HEIGHT,
            VisionSim.minTargetArea);
    simBack =
        new SimVisionSystem(
            Vision.BACK_CAMERA,
            VisionSim.camDiagFOVDegrees,
            Vision.ROBOT_TO_BACK_CAM,
            VisionSim.maxLEDRangeMeters,
            VisionSim.CAMERA_RES_WIDTH,
            VisionSim.CAMERA_RES_HEIGHT,
            VisionSim.minTargetArea);
             
    simFront.addVisionTargets(tagLayout);
    simBack.addVisionTargets(tagLayout);

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
