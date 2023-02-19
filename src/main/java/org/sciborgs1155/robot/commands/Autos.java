package org.sciborgs1155.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;
import org.sciborgs1155.lib.Vision;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.subsystems.Drive;

public final class Autos implements Sendable {

  private final Vision vision;
  private final Drive drive;
  private final SendableChooser<Command> chooser;

  public Autos(Drive drive) {
    vision = new Vision();
    this.drive = drive;
    chooser = new SendableChooser<>();
    chooser.setDefaultOption("mobility", mobility());
    chooser.addOption("other", drive.follow("New Path", true, false));
  }

  public Command get() {
    return chooser.getSelected();
  }

  private Command mobility() {
    return Commands.run(() -> drive.drive(0.5, 0, 0, false), drive).withTimeout(5);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    chooser.initSendable(builder);
  }
  // public Command cameraAlignment() {

  //   var result = cam.getLatestResult();
  //   PhotonTrackedTarget target = result.getBestTarget();
  //   Transform3d bestCameraToTarget = target.getBestCameraToTarget();
  //   double xp = bestCameraToTarget.getX();
  //   double yp = bestCameraToTarget.getY();
  //   double currentX = odometry.getEstimatedPosition().getX();
  //   double currentY = odometry.getEstimatedPosition().getY();
  //   double xpole = xp + currentX;
  //   double ypole = yp + currentY;
  //   return align(xpole, ypole);
  // }
  // public Command align(double xpole, double ypole) {
  //   PathPlannerTrajectory trajectory;
  //   return Commands.run(
  //     () -> drive.follow(PathPlannerTrajectory trajectory));
  // }

  public List<PathPoint> generatePointList(double xpole, double ypole) {
    Translation2d pointPosition = new Translation2d(xpole, ypole);
    PathPoint polePoint = new PathPoint(pointPosition, new Rotation2d(0), new Rotation2d(0), 0);
    List<PathPoint> pointList = new ArrayList<PathPoint>();
    pointList.add(polePoint);
    return pointList;
  }

  public PathPlannerTrajectory generateTrajectory(double xpole, double ypole) {
    PathConstraints trajectoryConstraints =
        new PathConstraints(Constants.Auto.MAX_SPEED, Constants.Auto.MAX_ACCEL);
    PathPlannerTrajectory trajectory =
        PathPlanner.generatePath(trajectoryConstraints, generatePointList(xpole, ypole));
    return trajectory;
  }

  public Command align(double xpole, double ypole) {
    PathPlannerTrajectory trajectory = generateTrajectory(xpole, ypole);
    return drive.follow(trajectory, false, false);
  }
}
