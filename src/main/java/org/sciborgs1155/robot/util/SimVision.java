package org.sciborgs1155.robot.util;

import static org.sciborgs1155.robot.Constants.Vision.*;

import edu.wpi.first.math.geometry.Pose2d;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.SimVisionSystem;
import org.photonvision.targeting.PhotonTrackedTarget;

public class SimVision implements VisionIO {
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

  private final RealVision realVision = new RealVision();

  public SimVision() {
    simFront.addVisionTargets(realVision.getLayout());
    simBack.addVisionTargets(realVision.getLayout());
  }

  @Override
  public void close() throws Exception {
    realVision.close();
  }

  @Override
  public boolean hasTargets() {
    return realVision.hasTargets();
  }

  @Override
  public PhotonTrackedTarget getBestTarget() {
    return realVision.getBestTarget();
  }

  @Override
  public EstimatedRobotPose[] getPoseEstimates(Pose2d lastPose) {
    simFront.processFrame(lastPose);
    simBack.processFrame(lastPose);
    ;

    return realVision.getPoseEstimates(lastPose);
  }

  @Override
  public void updateSeenTags() {
    realVision.updateSeenTags();
  }
}
