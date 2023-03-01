package org.sciborgs1155.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.subsystems.Arm;
import org.sciborgs1155.robot.subsystems.Drive;
import org.sciborgs1155.robot.subsystems.Elevator;

public final class Scoring {
  private final Drive drive;
  public final Arm arm;
  public final Elevator elevator;

  public Scoring(Drive drive, Arm arm, Elevator elevator) {
    this.drive = drive;
    this.arm = arm;
    this.elevator = elevator;
  }

  public Command align(Translation3d target) {
    var destination = new ArrayList<PathPoint>();
    destination.add(
        new PathPoint(new Translation2d(drive.getPose().getX(), target.getY()), new Rotation2d()));
    return drive.follow(
        PathPlanner.generatePath(
            new PathConstraints(Constants.Auto.MAX_SPEED, Constants.Auto.MAX_ACCEL), destination),
        false,
        false);
  }
}
