// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.lib.sim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import org.photonvision.SimVisionSystem;
import org.sciborgs1155.robot.Constants.Vision;

/** Add your docs here. */
public class VisionSim {
  public static final double maxLEDRangeMeters = 9000;
  public static final double camDiagFOVDegrees = 68.5;
  public static final double minTargetArea = 90;
  public static final int CAMERA_RES_WIDTH = 960;
  public static final int CAMERA_RES_HEIGHT = 544;

  AprilTagFieldLayout aprilTagFieldLayout =
      new AprilTagFieldLayout(
          Vision.AprilTagPose.APRIL_TAGS, Vision.FIELD_LENGTH, Vision.FIELD_WIDTH);
  // TODO change constants when received (damn you, construction)
  private static final SimVisionSystem visionSim =
      new SimVisionSystem(
          Vision.CAM_NAME,
          camDiagFOVDegrees,
          Vision.ROBOT_TO_CAM,
          maxLEDRangeMeters,
          CAMERA_RES_WIDTH,
          CAMERA_RES_HEIGHT,
          minTargetArea);

  private VisionSim() {
    visionSim.addVisionTargets(aprilTagFieldLayout);
  }

  public static SimVisionSystem getInstance() {
    return visionSim;
  }
}
