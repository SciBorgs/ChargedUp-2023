package org.sciborgs1155.lib;

import org.photonvision.PhotonCamera;
import org.sciborgs1155.robot.Constants.Vision;

/** Add your docs here. */
public class Camera {
  private static final PhotonCamera cam = new PhotonCamera(Vision.CAM_NAME);

  private Camera() {}

  public static PhotonCamera getInstance() {
    return cam;
  }
}
