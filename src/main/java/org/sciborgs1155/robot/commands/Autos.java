package org.sciborgs1155.robot.commands;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.sciborgs1155.lib.PlacementState;
import org.sciborgs1155.lib.Vision;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.subsystems.Arm;
import org.sciborgs1155.robot.subsystems.Drive;
import org.sciborgs1155.robot.subsystems.Elevator;
import org.sciborgs1155.robot.subsystems.Intake;

public class Autos implements Sendable {
  private final SendableChooser<AutoPath> pathChooser;

  public Autos(Drive drive, Intake intake, Vision vision, Arm arm, Elevator elevator) {
    pathChooser = new SendableChooser<AutoPath>();
    pathChooser.setDefaultOption("ex", AutoPath.examplePath(drive, vision, intake, arm, elevator));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    pathChooser.initSendable(builder);
  }

  public Command get() {
    return pathChooser.getSelected().get();
  }

  public final class PlaceHolderCommands {
    public static Command score(
        Intake intake,
        Drive drive,
        Elevator elevator,
        Arm arm,
        Vision vision,
        ShouldBeInDiffFile.GamePiece gamePiece,
        PlacementState scoringState) {
      return Commands.none();
    }

    public static Command intake(
        Intake intake, Arm arm, Elevator elevator, ShouldBeInDiffFile.GamePiece gamePiece) {
      return Commands.none();
    }
  }

  public final class ShouldBeInDiffFile {
    public enum ScoringHeight {
      HIGH,
      MID,
      LOW;
    }

    public enum GamePiece {
      CONE,
      CUBE
    }

    public enum RobotSide {
      FRONT,
      BACK
    }

    public static PlacementState scoringState(
        GamePiece gamePiece, ScoringHeight height, RobotSide side) {
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
}
