package org.sciborgs1155.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

public interface VisionIO extends AutoCloseable {
  /**
   * Returns whether the camera(s) have a target(s).
   *
   * @return Whether the camera(s) have a target(s).
   */
  public boolean hasTargets();

  /**
   * Returns the best target.
   *
   * @return The best target.
   */
  public PhotonTrackedTarget getBestTarget();

  /**
   * Returns estimated pose from vision measurements.
   *
   * @param lastPose
   * @return The estimated pose.
   */
  public EstimatedRobotPose[] getPoseEstimates(Pose2d lastPose);

  /**
   * Update dashboard with new tags.
   *
   * @return
   */
  public void updateSeenTags();
}
