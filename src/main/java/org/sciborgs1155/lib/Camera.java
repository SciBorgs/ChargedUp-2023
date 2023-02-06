package org.sciborgs1155.lib;

import org.photonvision.PhotonCamera;
import org.sciborgs1155.robot.Constants;

/** Add your docs here. */
public class Camera {
  private static final PhotonCamera cam = new PhotonCamera(Constants.Vision.CAMERA_NAME);

  private Camera() {}
  
  public static PhotonCamera getInstance() {
    return cam;
  }
}
