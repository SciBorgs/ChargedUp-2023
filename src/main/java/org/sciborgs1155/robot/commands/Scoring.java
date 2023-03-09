package org.sciborgs1155.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import org.sciborgs1155.lib.PlacementState;
import org.sciborgs1155.lib.Vision;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.subsystems.Drive;
import org.sciborgs1155.robot.subsystems.Intake;

public class Scoring {
  private final Intake intake;
  private final Drive drive;
  private final Placement placement;
  private final Vision vision;

  public Scoring(Drive drive, Placement placement, Intake intake, Vision vision) {
    this.intake = intake;
    this.drive = drive;
    this.placement = placement;
    this.vision = vision;
  }

  public Command score(GamePiece gamePiece, ScoringHeight height, Side side) {
    if (height == ScoringHeight.HIGH && side == Side.BACK) {
      throw new RuntimeException("cannot score high in the back");
    }
    return placement
        .toState(scoringState(gamePiece, height, side))
        .andThen(intake.start(true))
        .andThen(Commands.waitSeconds(3))
        .andThen(intake.stop());
  }

  // make it take gamePiece into account (maybe need to change format of field constants)
  public Command odometryAlign(Side side, Color color) {
    return drive.driveToPose(closestScoringPoint(side, color));
  }

  private Pose2d closestScoringPoint(Side side, Color color) {
    Collection<Translation2d> scoringPoints = Constants.Field.SCORING_POINTS.values();
    Translation2d point =
        drive
            .getPose()
            .getTranslation()
            .nearest(new ArrayList<Translation2d>(List.copyOf(scoringPoints)));
    // blue front: 180; blue back: 0; red front: 0; red front: 180
    double rotationDeg = 180;
    if (side == Side.BACK) {
      rotationDeg = (rotationDeg + 180) % 360;
    }
    if (color == Color.RED) {
      rotationDeg = (rotationDeg + 180) % 360;
    }
    return new Pose2d(point, Rotation2d.fromDegrees(rotationDeg));
  }

  public enum Side {
    BACK,
    FRONT
  }

  public enum Color {
    RED,
    BLUE
  }

  public enum GamePiece {
    CONE,
    CUBE
  }

  public enum ScoringHeight {
    HIGH,
    MID,
    LOW
  }

  public static PlacementState scoringState(GamePiece gamePiece, ScoringHeight height, Side side) {
    switch (gamePiece) {
      case CONE:
        switch (height) {
          case HIGH:
            switch (side) {
              case FRONT:
                return Constants.Positions.FRONT_HIGH_CONE;
              case BACK:
                return Constants.Positions.BACK_HIGH_CONE;
            }
          case MID:
            switch (side) {
              case FRONT:
                return Constants.Positions.FRONT_MID_CONE;
              case BACK:
                return Constants.Positions.BACK_MID_CONE;
            }
          case LOW:
            return Constants.Positions.BACK_LOW_CONE;
        }
      case CUBE:
        switch (height) {
          case HIGH:
            switch (side) {
              case FRONT:
                return Constants.Positions.FRONT_HIGH_CUBE;
              case BACK:
                return Constants.Positions.BACK_HIGH_CUBE;
            }
          case MID:
            switch (side) {
              case FRONT:
                return Constants.Positions.FRONT_MID_CUBE;
              case BACK:
                return Constants.Positions.BACK_MID_CUBE;
            }
          case LOW:
            return Constants.Positions.BACK_LOW_CUBE;
        }
    }
    throw new RuntimeException(
        "scoringState was not called on a valid arguments. \n"
            + "gamePiece: "
            + gamePiece
            + "; height: "
            + height
            + "; side: "
            + side);
  }
}
