package org.sciborgs1155.robot.commands;

import static org.sciborgs1155.robot.Constants.Field.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Collection;
import java.util.List;
import org.sciborgs1155.robot.subsystems.Drive;
import org.sciborgs1155.robot.util.placement.PlacementState.Side;

public class Alignment {

  private final Drive drive;
  private final Scoring scoring;

  public Alignment(Drive drive, Scoring scoring) {
    this.drive = drive;
    this.scoring = scoring;
  }

  public Command odometryAlign() {
    return odometryAlign(drive.getPose(), scoring.side());
  }

  public Command odometryAlign(Pose2d startPose, Side side) {
    return drive.driveToPose(startPose, closestScoringPoint(startPose, side), true);
  }

  private Pose2d closestScoringPoint(Pose2d pose, Side side) {
    Collection<Translation2d> scoringPoints =
        switch (scoring.gamePiece()) {
          case CONE -> SCORING_POINTS_CONE.values();
          case CUBE -> SCORING_POINTS_CUBE.values();
        };
    Translation2d point = pose.getTranslation().nearest(List.copyOf(scoringPoints));
    return new Pose2d(point, Rotation2d.fromRadians(side.rads()));
  }
}
