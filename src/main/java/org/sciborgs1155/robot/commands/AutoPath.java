package org.sciborgs1155.robot.commands;

import java.util.ArrayList;
import java.util.List;

import org.sciborgs1155.lib.State;
import org.sciborgs1155.lib.Vision;
import org.sciborgs1155.robot.subsystems.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import org.sciborgs1155.robot.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoPath implements Sendable {

    private final SendableChooser<Command> chooser;
    private final Drive drive;
    private final Vision vision;
    private final Intake intake;
    private final Arm arm;
    private final Elevator elevator;

    public AutoPath(Drive drive, Vision vision, Intake intake, Arm arm, Elevator elevator) {
        chooser = new SendableChooser<Command>();
        chooser.setDefaultOption("none", Commands.none());
        this.drive = drive;
        this.vision = vision;
        this.intake = intake;
        this.arm = arm;
        this.elevator = elevator;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        chooser.initSendable(builder);
    }

    public final class AutoCommands {

        public Command goToPoint(PathPoint endPoint) {
            Pose2d pose = drive.getPose();
            PathPoint currentPoint = new PathPoint(pose.getTranslation(), pose.getRotation(), pose.getRotation());
            PathConstraints constraints = new PathConstraints(Constants.Auto.MAX_SPEED, Constants.Auto.MAX_ACCEL);
            List<PathPoint> points = new ArrayList<PathPoint>(List.of(currentPoint, endPoint));
            PathPlannerTrajectory trajectory = PathPlanner.generatePath(constraints, points);
            return drive.follow(trajectory, false, false); // should it be using alliance color?
        }

        public Command intake(GamePiece gamePiece) {
            return PlaceHolderCommands.intake(intake, arm, elevator, gamePiece);
        }
    }

    public final class PlaceHolderCommands {
        public static Command score(Intake intake, Drive drive, Elevator elevator,
                             Arm arm, Vision vision, GamePiece gamePiece,
                             State scoringState) {
            return Commands.none();
        }

        public static Command intake(Intake intake, Arm arm, Elevator elevator, GamePiece gamePiece) {
            return Commands.none();
        }
    }

    public enum ScoringHeight {
        HIGH,
        MID,
        LOW;

        public State scoringState(GamePiece gamePiece) {
            switch (gamePiece) {
                case CONE:
                    switch (this) {
                        case HIGH: return Constants.Placement.HIGH_CONE;                   
                        case MID: return Constants.Placement.MID_CONE;
                        case LOW: return Constants.Placement.LOW_CONE;
                    }
                case CUBE:
                    switch (this) {
                        case HIGH: return Constants.Placement.HIGH_CUBE;
                        case MID: return Constants.Placement.MID_CUBE;
                        case LOW: return Constants.Placement.LOW_CUBE;
                    }
            }
            throw new RuntimeException("scoringState was note called on valid arguments... probably...");
        }
    }

    public enum GamePiece {
        CONE,
        CUBE
    }
}
