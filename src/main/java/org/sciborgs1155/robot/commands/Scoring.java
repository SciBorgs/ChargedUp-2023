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
import java.util.Collection;
import java.util.List;
import org.sciborgs1155.lib.DeferredCommand;
import org.sciborgs1155.robot.subsystems.Drive;
import org.sciborgs1155.robot.subsystems.LED;
import org.sciborgs1155.robot.util.placement.PlacementState;

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

  public Command odometryAlign(Side side) {
    return odometryAlign(drive.getPose(), side);
  }

  public Command odometryAlign(Pose2d startPose, Side side) {
    return drive.driveToPose(startPose, closestScoringPoint(startPose, side), true);
  }

  private Pose2d closestScoringPoint(Pose2d pose, Side side) {
    Collection<Translation2d> scoringPoints =
        switch (gamePiece) {
          case CONE -> SCORING_POINTS_CONE.values();
          case CUBE -> SCORING_POINTS_CUBE.values();
        };
    Translation2d point = pose.getTranslation().nearest(List.copyOf(scoringPoints));
    return new Pose2d(point, Rotation2d.fromRadians(side.rads()));
  }

  public Command goTo(Level height) {
    return new DeferredCommand(() -> placement.goTo(scoringState(height)));
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
