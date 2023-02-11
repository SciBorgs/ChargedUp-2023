// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.commands;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.subsystems.Drive;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;


public final class Autos implements Sendable {

  private final Drive drive;
  private final PhotonCamera cam;
  private final SendableChooser<Command> chooser;
  private final SwerveDrivePoseEstimator odometry;

  public Autos(Drive drive, PhotonCamera cam, SwerveDrivePoseEstimator odometry) {
    this.drive = drive;
    this.cam = cam;
    this.odometry = odometry;

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

  public Command cameraAlignment() {
    var result = cam.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    Transform3d bestCameraToTarget = target.getBestCameraToTarget();
    double xp = bestCameraToTarget.getX();
    double yp = bestCameraToTarget.getY();
    double currentX = odometry.getEstimatedPosition().getX();
    double currentY = odometry.getEstimatedPosition().getY();
    double xpole = xp + currentX;
    double ypole = yp + currentY;
    return align(xpole, ypole); 
  }

  public PathPoint pathPoint(double xpole, double ypole){
    Translation2d pointPosition = new Translation2d(xpole, ypole);
    Rotation2d heading = new Rotation2d(0);
    Rotation2d holonomicRotation = new Rotation2d(0);
    return new PathPoint(pointPosition, new Rotation2d(0), new Rotation2d(0), 0);
  }
  public PathPlannerTrajectory trajectory(){
  public final PathConstraints trajectoryConstraints = new PathConstraints(Constants.Auto.MAX_SPEED, Constants.Auto.MAX_ACCEL);
  return new PathPlannerTrajectory() trajectory = PathPlanner.generatePath(trajectoryConstraints, PathPoint());
  }


  public Command align(double xpole, double ypole) {
    PathPlannerTrajectory trajectory = new PathPlannerTrajectory();
    return drive.follow(trajectory, false, false);
  }
  
}