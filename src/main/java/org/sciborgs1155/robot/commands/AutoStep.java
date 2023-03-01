package org.sciborgs1155.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import org.sciborgs1155.lib.PlacementState;
import org.sciborgs1155.lib.Vision;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.commands.Autos.PlaceHolderCommands;
import org.sciborgs1155.robot.commands.Autos.ShouldBeInDiffFile;
import org.sciborgs1155.robot.commands.Autos.ShouldBeInDiffFile.*;
import org.sciborgs1155.robot.subsystems.Arm;
import org.sciborgs1155.robot.subsystems.Drive;
import org.sciborgs1155.robot.subsystems.Elevator;
import org.sciborgs1155.robot.subsystems.Intake;

public interface AutoStep extends Sendable {

  @Override
  void initSendable(SendableBuilder builder);

  Command get();

  public final class Score implements AutoStep {
    private final SendableChooser<GamePiece> gamePieceChooser;
    private final SendableChooser<ScoringHeight> scoringHeightChooser;
    private final Pose2d scoringPose;

    private RobotSide robotSide;

    // subsystems
    private final Drive drive;
    private final Vision vision;
    private final Intake intake;
    private final Arm arm;
    private final Elevator elevator;

    public Score(
        Pose2d scoringPose,
        RobotSide robotSide,
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
      robotSide =
          scoringHeightChooser.getSelected() == ScoringHeight.LOW ? robotSide : RobotSide.BACK;

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
      intakePoseChooser.setDefaultOption("red 1", Constants.Field.IntakePoints.RED_ONE);
      intakePoseChooser.addOption("red 2", Constants.Field.IntakePoints.RED_TWO);
      intakePoseChooser.addOption("red 3", Constants.Field.IntakePoints.RED_THREE);
      intakePoseChooser.addOption("red 4", Constants.Field.IntakePoints.RED_FOUR);
      intakePoseChooser.addOption("blue 1", Constants.Field.IntakePoints.BLUE_ONE);
      intakePoseChooser.addOption("blue 2", Constants.Field.IntakePoints.BLUE_TWO);
      intakePoseChooser.addOption("blue 3", Constants.Field.IntakePoints.BLUE_THREE);
      intakePoseChooser.addOption("blue 4", Constants.Field.IntakePoints.BLUE_FOUR);

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

  public final class DriveToPose implements AutoStep {
    private Pose2d pose;

    private final Drive drive;

    public DriveToPose(Drive drive, Pose2d pose) {
      this.pose = pose;
      this.drive = drive;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("x coord", () -> this.pose.getX(), this::setX);
        builder.addDoubleProperty("y coord", () -> this.pose.getY(), this::setY);
        builder.addDoubleProperty("theta (rads)", () -> this.pose.getRotation().getRadians(), this::setTheta);
    }

    @Override
    public Command get() {
        return drive.driveToPose(pose);
    }

    private void setX(double x) {
      this.pose = new Pose2d(x, this.pose.getY(), this.pose.getRotation());
    }

    private void setY(double y) {
      this.pose = new Pose2d(this.pose.getX(), y, this.pose.getRotation());
    }

    private void setTheta(double thetaRads) {
      this.pose = new Pose2d(this.pose.getTranslation(), Rotation2d.fromRadians(thetaRads));
    }
  }
}
