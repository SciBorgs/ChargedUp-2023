package org.sciborgs1155.robot.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class VisionConstants {
  // Camera names
  public static final String FRONT_CAM = "frontPhotonVision";
  public static final String BACK_CAM = "backPhotonVision";

  // Robot to camera translations
  public static final Translation3d FRONT_CAM_TRANSLATION = new Translation3d(0.165, -0.305, 0.356);
  public static final Rotation3d FRONT_CAM_ROTATION = new Rotation3d(1.64933614, 1.78, 0);
  public static final Transform3d ROBOT_TO_FRONT_CAM =
      new Transform3d(FRONT_CAM_TRANSLATION, FRONT_CAM_ROTATION);

  public static final Translation3d BACK_CAM_TRANSLATION =
      new Translation3d(); // Units.degreesToRadians(-180)
  public static final Rotation3d BACK_CAM_ROTATION =
      new Rotation3d(0, 0, Units.degreesToRadians(-180));
  public static final Transform3d ROBOT_TO_BACK_CAM =
      new Transform3d(BACK_CAM_TRANSLATION, BACK_CAM_ROTATION);

  public static final PoseStrategy PRIMARY_POSE_STRATEGY = PoseStrategy.LOWEST_AMBIGUITY;
  public static final PoseStrategy SECONDARY_POSE_STRATEGY = PoseStrategy.LOWEST_AMBIGUITY;

  public static final double LED_RANGE = 9000;
  public static final double CAM_FOV = 68.5;
  public static final double MIN_TARGET_AREA = 90;
  public static final int CAMERA_RES_WIDTH = 960;
  public static final int CAMERA_RES_HEIGHT = 544;
}
