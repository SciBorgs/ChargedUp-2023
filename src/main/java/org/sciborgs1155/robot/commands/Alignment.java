package org.sciborgs1155.robot.commands;

import static org.sciborgs1155.robot.Constants.Field.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Collection;
import java.util.List;
import org.sciborgs1155.robot.subsystems.Drive;
import org.sciborgs1155.robot.subsystems.arm.ArmState.GamePiece;
import org.sciborgs1155.robot.subsystems.arm.ArmState.Side;

public class Alignment {

  private final Drive drive;

  public Alignment(Drive drive) {
    this.drive = drive;
  }

  public Command odometryAlign(Pose2d startPose, GamePiece gamePiece, Side side) {
    return drive.driveToPose(startPose, closestScoringPoint(startPose, gamePiece, side), true);
  }

  private Pose2d closestScoringPoint(Pose2d pose, GamePiece gamePiece, Side side) {
    Collection<Translation2d> scoringPoints =
        switch (gamePiece) {
          case CONE -> SCORING_POINTS_CONE.values();
          case CUBE -> SCORING_POINTS_CUBE.values();
        };
    Translation2d point = pose.getTranslation().nearest(List.copyOf(scoringPoints));
    return new Pose2d(point, Rotation2d.fromRadians(side.angle));
  }
}
