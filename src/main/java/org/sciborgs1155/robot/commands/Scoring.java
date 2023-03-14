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
import org.sciborgs1155.robot.Constants.*;
import org.sciborgs1155.robot.subsystems.Drive;
import org.sciborgs1155.robot.subsystems.Intake;

public class Scoring {
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

  public enum Alliance {
    RED,
    BLUE;

    public double rads() {
      if (this == RED) {
        return Math.PI;
      } else return 0;
    }
  }

  public enum GamePiece {
    NONE,
    CONE,
    CUBE
  }

  public enum ScoringHeight {
    HIGH,
    MID,
    LOW
  }

  private final Intake intake;
  private final Drive drive;
  private final Placement placement;
  private final Vision vision;

  private GamePiece gamePiece;

  public Scoring(Drive drive, Placement placement, Intake intake, Vision vision) {
    this.intake = intake;
    this.drive = drive;
    this.placement = placement;
    this.vision = vision;

    this.gamePiece = GamePiece.CUBE;
  }

  public Command score(ScoringHeight height, Side side) {
    return score(gamePiece, height, side);
  }

  private Command score(GamePiece gamePiece, ScoringHeight height, Side side) {
    if (height == ScoringHeight.HIGH && side == Side.FRONT && gamePiece == GamePiece.CONE) {
      throw new RuntimeException("cannot score a cone high in the front");
    }
    return placement.toState(scoringState(gamePiece, height, side)).andThen(intake.outtake());
  }

  // TODO leds!
  public Command setGamePiece(GamePiece gamePiece) {
    return Commands.runOnce(() -> this.gamePiece = gamePiece);
  }

  // not sure if this is more helpful
  public Command toggleGamePiece() {
    return gamePiece == GamePiece.CONE
        ? setGamePiece(GamePiece.CUBE)
        : setGamePiece(GamePiece.CONE);
  }

  // TODO make it take gamePiece into account
  public Command odometryAlign(Side side, Alliance color) {
    return drive.driveToPose(drive.getPose(), closestScoringPoint(side, color));
  }

  // TODO make commands to go to the next scoring poses to the left and right

  // TODO vision alignment

  private Pose2d closestScoringPoint(Side side, Alliance color) {
    Collection<Translation2d> scoringPoints = Field.SCORING_POINTS.values();
    Translation2d point =
        drive
            .getPose()
            .getTranslation()
            .nearest(new ArrayList<Translation2d>(List.copyOf(scoringPoints)));
    double rotationRad = (side.rads() + color.rads()) % (2 * Math.PI);
    return new Pose2d(point, Rotation2d.fromRadians(rotationRad));
  }

  public Command intake(Side side, GamePiece gamePiece) {
    return placement.toState(intakeState(gamePiece, side)).andThen(intake.intake());
  }

  // this might end up being the right thing to do
  public Command intake(PlacementState intakeState) {
    return placement.toState(intakeState).andThen(intake.intake());
  }

  public static PlacementState scoringState(GamePiece gamePiece, ScoringHeight height, Side side) {
    switch (gamePiece) {
      case CONE:
        switch (height) {
          case HIGH:
            return Positions.BACK_HIGH_CONE;
          case MID:
            switch (side) {
              case FRONT:
                return Positions.FRONT_MID_CONE;
              case BACK:
                return Positions.BACK_MID_CONE;
            }
          case LOW:
            return Positions.BACK_LOW_CONE;
        }
      case CUBE:
        switch (height) {
          case HIGH:
            switch (side) {
              case FRONT:
                return Positions.FRONT_HIGH_CUBE;
              case BACK:
                return Positions.BACK_HIGH_CUBE;
            }
          case MID:
            switch (side) {
              case FRONT:
                return Positions.FRONT_MID_CUBE;
              case BACK:
                return Positions.BACK_MID_CUBE;
            }
          case LOW:
            return Positions.BACK_LOW_CUBE;
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

  // we probably don't need this
  public static PlacementState intakeState(GamePiece gamePiece, Side side) {
    // TODO either get rid of this or make it actually correct
    switch (side) {
      case BACK:
        return Positions.BACK_INTAKE;
      case FRONT:
        return Positions.FRONT_INTAKE;
    }
    throw new RuntimeException(
        "intakeState was not called on a valid arguments. \n"
            + "gamePiece: "
            + gamePiece
            + "; side: "
            + side);
  }
}
