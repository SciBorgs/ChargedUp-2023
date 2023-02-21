package org.sciborgs1155.lib;

import static org.sciborgs1155.robot.Constants.Vision.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.Optional;
import java.util.stream.Stream;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.SimVisionSystem;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.sciborgs1155.robot.Robot;

public class Vision {

  public enum Mode {
    REAL,
    SIM,
    SIM_WITH_CAM;
  }

  private final Mode mode;

  private final PhotonCamera frontCam = new PhotonCamera(FRONT_CAM);
  private final PhotonCamera backCam = new PhotonCamera(BACK_CAM);

  private final PhotonPoseEstimator frontEstimator;
  private final PhotonPoseEstimator backEstimator;

  private AprilTagFieldLayout layout;

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

  public Vision() {
    this(Robot.isReal() ? Mode.REAL : Mode.SIM);
  }

  public Vision(Mode mode) {
    try {
      layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();

      // Load tags in AdvantageScope
      if (mode == Mode.SIM) {
        AprilTag[] allTags = layout.getTags().toArray(new AprilTag[] {});
        SmartDashboard.putNumberArray("tagposes", createObjects(allTags));

        double[] list = new double[allTags.length];
        for (int i = 0; i < allTags.length; i++) list[i] = (double) i;
        SmartDashboard.putNumberArray("tag IDs", list);
      }
    } catch (Exception e) {
      layout = new AprilTagFieldLayout(new ArrayList<AprilTag>(), 0, 0);
      DriverStation.reportError(
          "Failed to load AprilTagFieldLayout, odometry will not work correctly",
          e.getStackTrace());
    }

    frontEstimator =
        new PhotonPoseEstimator(layout, PRIMARY_POSE_STRATEGY, frontCam, ROBOT_TO_FRONT_CAM);
    backEstimator =
        new PhotonPoseEstimator(layout, PRIMARY_POSE_STRATEGY, backCam, ROBOT_TO_BACK_CAM);

    simFront.addVisionTargets(layout);
    simBack.addVisionTargets(layout);

    frontEstimator.setMultiTagFallbackStrategy(SECONDARY_POSE_STRATEGY);
    backEstimator.setMultiTagFallbackStrategy(SECONDARY_POSE_STRATEGY);

    this.mode = mode;

    // set up networktables if we are in hardware sim mode
    if (mode == Mode.SIM_WITH_CAM) {
      if (Robot.isReal()) {
        throw new IllegalStateException(
            "The robot is real and cannot be used in mode SIM_WITH_CAM");
      }
      var inst = NetworkTableInstance.getDefault();
      inst.stopServer();
      inst.setServer("photonvision.local");
      inst.startClient4("Robot Simulation");
    }
  }

  private double[] createObjects(AprilTag[] scopeTags) {
    double[] data = new double[scopeTags.length * 7];
    for (int i = 0; i < scopeTags.length; i++) {
      data[i * 7] = scopeTags[i].pose.getX();
      data[i * 7 + 1] = scopeTags[i].pose.getY();
      data[i * 7 + 2] = scopeTags[i].pose.getZ();
      data[i * 7 + 3] = scopeTags[i].pose.getRotation().getQuaternion().getW();
      data[i * 7 + 4] = scopeTags[i].pose.getRotation().getQuaternion().getX();
      data[i * 7 + 5] = scopeTags[i].pose.getRotation().getQuaternion().getY();
      data[i * 7 + 6] = scopeTags[i].pose.getRotation().getQuaternion().getZ();
    }
    return data;
  }

  /* TODO refactor
   * Determines if and which tags are in view (AdvantageScope sim)
   ** When we get cam positions, cam override allows rays to originate from their real positions, not just center (nvm it cant even do this as it is)
   */
  public AprilTag[] determineSeenTags(PhotonCamera cam, AprilTagFieldLayout layout) {
    ArrayList<AprilTag> tags = new ArrayList<>();

    if (cam.getLatestResult().hasTargets()) {
      var targets = cam.getLatestResult().getTargets();
      for (PhotonTrackedTarget tag : targets) {
        if (tag.getFiducialId() != -1) {
          int id = tag.getFiducialId();
          tags.add(new AprilTag(id, layout.getTagPose(id).get()));
        }
      }
    }
    return tags.toArray(new AprilTag[] {});
  }

  public void updateSeenTags() {
    SmartDashboard.putNumberArray(
        "visible front tags", createObjects(determineSeenTags(frontCam, layout)));
    SmartDashboard.putNumberArray(
        "visible back tags", createObjects(determineSeenTags(backCam, layout)));
  }
  /* Gets estimated pose from vision measurements */
  public EstimatedRobotPose[] getPoseEstimates(Pose2d lastPose) {
    frontEstimator.setReferencePose(lastPose);
    backEstimator.setReferencePose(lastPose);

    if (mode == Mode.SIM) {
      simFront.processFrame(lastPose);
      simBack.processFrame(lastPose);
    }

    return Stream.of(frontEstimator, backEstimator)
        .map(PhotonPoseEstimator::update)
        .flatMap(Optional::stream)
        .toArray(EstimatedRobotPose[]::new);
  }
}
