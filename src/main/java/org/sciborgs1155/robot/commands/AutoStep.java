package org.sciborgs1155.robot.commands;

import org.sciborgs1155.lib.State;
import org.sciborgs1155.lib.Vision;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.commands.AutoPath.*;
import org.sciborgs1155.robot.subsystems.Arm;
import org.sciborgs1155.robot.subsystems.Drive;
import org.sciborgs1155.robot.subsystems.Elevator;
import org.sciborgs1155.robot.subsystems.Intake;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public interface AutoStep extends Sendable {

    @Override
    void initSendable(SendableBuilder builder);

    Command get();

    public final class Score implements AutoStep {
        private final SendableChooser<GamePiece> gamePieceChooser;
        private final SendableChooser<ScoringHeight> scoringHeightChooser;
        private final Pose2d scoringPose;

        // subsystems
        private final Drive drive;
        private final Vision vision;
        private final Intake intake;
        private final Arm arm;
        private final Elevator elevator;

        public Score(Pose2d scoringPose, Drive drive, Vision vision, Intake intake, Arm arm, Elevator elevator) {
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
        }

        @Override 
        public void initSendable(SendableBuilder builder) {
            gamePieceChooser.initSendable(builder);
            scoringHeightChooser.initSendable(builder);
        }

        @Override
        public Command get() {
            GamePiece gamePiece = gamePieceChooser.getSelected();
            State scoringState = scoringHeightChooser.getSelected().scoringState(gamePiece);
            return drive.driveToPose(scoringPose).
                   andThen(PlaceHolderCommands.score(intake, drive, elevator, arm, vision, gamePiece, scoringState));
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
            return drive.driveToPose(intakePoseChooser.getSelected())
                   .andThen(PlaceHolderCommands.intake(intake, null, null, null));
        }
    }
}
