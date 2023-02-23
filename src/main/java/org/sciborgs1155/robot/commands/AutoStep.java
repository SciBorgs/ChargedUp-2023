package org.sciborgs1155.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.commands.Autos.PlaceHolderCommands;
import org.sciborgs1155.robot.commands.Autos.ShouldBeInDiffFile;
import org.sciborgs1155.robot.commands.Autos.ShouldBeInDiffFile.*;
import org.sciborgs1155.robot.subsystems.Arm;
import org.sciborgs1155.robot.subsystems.Drive;
import org.sciborgs1155.robot.subsystems.Elevator;
import org.sciborgs1155.robot.subsystems.Intake;
import org.sciborgs1155.robot.util.PlacementState;
import org.sciborgs1155.robot.util.State.Side;
import org.sciborgs1155.robot.util.Vision;

public interface AutoStep extends Sendable {

  @Override
  void initSendable(SendableBuilder builder);

  Command get();

  public final class Score implements AutoStep {
    private final SendableChooser<GamePiece> gamePieceChooser;
    private final SendableChooser<ScoringHeight> scoringHeightChooser;
    private final Pose2d scoringPose;

    private Side robotSide;

    // subsystems
    private final Drive drive;
    private final Vision vision;
    private final Intake intake;
    private final Arm arm;
    private final Elevator elevator;

    public Score(
        Pose2d scoringPose,
        Side robotSide,
        Drive drive,
        Vision vision,
        Intake intake,
        Arm arm,
        Elevator elevator) {
      gamePieceChooser = new SendableChooser<GamePiece>();
      gamePieceChooser.setDefaultOption("cube", GamePiece.CUBE);
      gamePieceChooser.addOption("cone", GamePiece.CONE);

      scoringHeightChooser = new SendableChooser<ScoringHeight>();
      scoringHeightChooser.setDefaultOption("low", ScoringHeight.LOW);
      scoringHeightChooser.addOption("mid", ScoringHeight.MID);
      scoringHeightChooser.addOption("high", ScoringHeight.HIGH);

      this.scoringPose = scoringPose;
      this.drive = drive;
      this.vision = vision;
      this.intake = intake;
      this.arm = arm;
      this.elevator = elevator;
      this.robotSide = robotSide;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
      gamePieceChooser.initSendable(builder);
      scoringHeightChooser.initSendable(builder);
    }

    @Override
    public Command get() {
      // making sure we're only scoring low from the back
      robotSide = scoringHeightChooser.getSelected() == ScoringHeight.LOW ? robotSide : Side.BACK;

      GamePiece gamePiece = gamePieceChooser.getSelected();
      PlacementState scoringState =
          ShouldBeInDiffFile.scoringState(gamePiece, scoringHeightChooser.getSelected(), robotSide);
      return drive
          .driveToPose(scoringPose)
          .andThen(
              PlaceHolderCommands.score(
                  intake, drive, elevator, arm, vision, gamePiece, scoringState));
    }
  }

  public final class IntakePiece implements AutoStep {
    private final SendableChooser<GamePiece> gamePieceChooser;
    private final SendableChooser<Pose2d> intakePoseChooser;

    private final Drive drive;
    private final Intake intake;
    private final Elevator elevator;
    private final Arm arm;

    public IntakePiece(Drive drive, Intake intake, Elevator elevator, Arm arm) {
      gamePieceChooser = new SendableChooser<GamePiece>();
      gamePieceChooser.setDefaultOption("cube", GamePiece.CUBE);
      gamePieceChooser.addOption("cone", GamePiece.CONE);

      intakePoseChooser = new SendableChooser<Pose2d>();
      intakePoseChooser.setDefaultOption("red 1", Constants.Field.INTAKE_POINTS.get("R1"));
      intakePoseChooser.addOption("red 2", Constants.Field.INTAKE_POINTS.get("R2"));
      intakePoseChooser.addOption("red 3", Constants.Field.INTAKE_POINTS.get("R3"));
      intakePoseChooser.addOption("red 4", Constants.Field.INTAKE_POINTS.get("R4"));
      intakePoseChooser.addOption("blue 1", Constants.Field.INTAKE_POINTS.get("B1"));
      intakePoseChooser.addOption("blue 2", Constants.Field.INTAKE_POINTS.get("B2"));
      intakePoseChooser.addOption("blue 3", Constants.Field.INTAKE_POINTS.get("B3"));
      intakePoseChooser.addOption("blue 4", Constants.Field.INTAKE_POINTS.get("B4"));

      this.drive = drive;
      this.intake = intake;
      this.elevator = elevator;
      this.arm = arm;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
      gamePieceChooser.initSendable(builder);
      intakePoseChooser.initSendable(builder);
    }

    @Override
    public Command get() {
      return drive
          .driveToPose(intakePoseChooser.getSelected())
          .andThen(
              PlaceHolderCommands.intake(intake, arm, elevator, gamePieceChooser.getSelected()));
    }
  }
}
