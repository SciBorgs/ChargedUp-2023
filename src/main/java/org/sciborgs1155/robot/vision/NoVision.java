package org.sciborgs1155.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

public class NoVision implements VisionIO {
  @Override
  public boolean hasTargets() {
    return false;
  }

  @Override
  public PhotonTrackedTarget getBestTarget() {
    return null; // TODO funny
  }

  @Override
  public EstimatedRobotPose[] getPoseEstimates(Pose2d lastPose) {
    return new EstimatedRobotPose[0];
  }

  @Override
  public void updateSeenTags() {}

  @Override
  public void close() {}
}
