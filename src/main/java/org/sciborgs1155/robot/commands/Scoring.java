package org.sciborgs1155.robot.commands;

import static org.sciborgs1155.robot.Constants.Field.*;
import static org.sciborgs1155.robot.Constants.Positions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import org.sciborgs1155.robot.subsystems.Drive;
import org.sciborgs1155.robot.subsystems.LED;
import org.sciborgs1155.robot.util.PlacementState;
import org.sciborgs1155.robot.util.Vision;

public final class Scoring implements Sendable {

  public enum Side {
    BACK,
    FRONT;

    public double rads() {
      if (this == BACK) {
        return Math.PI;
      }
      return 0;
    }
  }

  public enum GamePiece {
    CONE,
    CUBE;
  }

  public enum Level {
    HIGH,
    MID,
    LOW,
    SINGLE_SUBSTATION,
    DOUBLE_SUBSTATION,
  }

  private final Drive drive;
  private final Placement placement;
  private final LED led;

  private Side side = Side.BACK;
  private GamePiece gamePiece = GamePiece.CONE;

  public Scoring(Drive drive, Placement placement, LED led) {
    this.drive = drive;
    this.placement = placement;
    this.led = led;
  }

  public Command setGamePiece(GamePiece gamePiece) {
    return Commands.runOnce(() -> this.gamePiece = gamePiece)
        .alongWith(led.setGamePieceColor(gamePiece));
  }

  public Command setSide(Side side) {
    return Commands.runOnce(() -> this.side = side);
  }

  // TODO make it take gamePiece into account
  public Command odometryAlign(Side side, Vision vision, GamePiece gamePiece) {
    return drive.driveToPose(drive.getPose(), closestScoringPoint(side, vision, gamePiece), true);
  }

  // TODO make commands to go to the next scoring poses to the left and right
  private Pose2d closestScoringPoint(Side side, Vision vision, GamePiece gamePiece) {
    if (vision.hasTargets()) {
      int tagID = vision.getBestTarget().getFiducialId();

      Translation2d scorePoint = new Translation2d();
      if (gamePiece.CONE.equals(GamePiece.CONE)) {
        scorePoint = SCORING_POINTS_CONE.get(tagID);
      } else if (gamePiece.CUBE.equals(GamePiece.CUBE)) {
        scorePoint = SCORING_POINTS_CUBE.get(tagID);
      } else {
        if (tagID < 4) scorePoint = INTAKE_POINTS.get(tagID);
      }
      return new Pose2d(scorePoint, Rotation2d.fromRadians(side.rads() % (2 * Math.PI)));
    }
    return new Pose2d();
  }
  /*
  private Pose2d closestScoringPoint(Side side) {
    Collection<Translation2d> scoringPoints = SCORING_POINTS.values();
    Translation2d point =
        drive
            .getPose()
            .getTranslation()
            .nearest(new ArrayList<Translation2d>(List.copyOf(scoringPoints)));
    double rotationRad = (side.rads()  TODO use path planner flip color.rads()) % (2 * Math.PI);
    return new Pose2d(point, Rotation2d.fromRadians(rotationRad));
  }
  */

  public Command goTo(Level height) {
    return new ProxyCommand(() -> placement.safeToState(scoringState(height)));
  }

  public PlacementState scoringState(Level height) {
    return switch (height) {
      case LOW -> side == Side.FRONT ? FRONT_INTAKE : BACK_INTAKE;
      case MID -> switch (gamePiece) {
        case CONE -> side == Side.FRONT ? FRONT_MID_CONE : BACK_MID_CONE;
        case CUBE -> side == Side.FRONT ? FRONT_MID_CUBE : BACK_MID_CUBE;
      };
      case HIGH -> switch (gamePiece) {
        case CONE -> BACK_HIGH_CONE;
        case CUBE -> side == Side.FRONT ? FRONT_HIGH_CUBE : BACK_HIGH_CUBE;
        default -> throw new IllegalArgumentException("Unexpected value: " + gamePiece);
      };
      case SINGLE_SUBSTATION -> switch (gamePiece) {
        case CONE -> FRONT_SINGLE_SUBSTATION_CONE;
        case CUBE -> FRONT_SINGLE_SUBSTATION_CUBE;
      };
      case DOUBLE_SUBSTATION -> BACK_DOUBLE_SUBSTATION;
    };
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addStringProperty("Game Piece", () -> gamePiece.name(), null);
    builder.addStringProperty("Side", () -> side.name(), null);
  }
}
