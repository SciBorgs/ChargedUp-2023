package org.sciborgs1155.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.subsystems.*;
import org.sciborgs1155.robot.util.State.Side;
import org.sciborgs1155.robot.util.Vision;

public class AutoPath {

  private final Drive drive;
  private final Vision vision;
  private final Intake intake;
  private final Arm arm;
  private final Elevator elevator;

  private List<AutoStep> steps;

  private AutoPath(
      Drive drive, Vision vision, Intake intake, Arm arm, Elevator elevator, List<AutoStep> steps) {
    this.drive = drive;
    this.vision = vision;
    this.intake = intake;
    this.arm = arm;
    this.elevator = elevator;
    this.steps = steps;
  }

  public static AutoPath examplePath(
      Drive drive, Vision vision, Intake intake, Arm arm, Elevator elevator) {
    AutoPath autoPath =
        new AutoPath(drive, vision, intake, arm, elevator, new ArrayList<AutoStep>());
    // autoPath.addIntakeStep();
    autoPath.addScoreStep(Constants.Field.SCORING_POINTS.get("B8"), Side.FRONT);
    return autoPath;
  }

  public static AutoPath simpleDrivePath(Drive drive, Vision vision, Intake intake, Arm arm, Elevator elevator) {
    AutoPath autoPath = new AutoPath(drive, vision, intake, arm, elevator, new ArrayList<AutoStep>());
    autoPath.addDriveStep(new Pose2d(0, 4, Rotation2d.fromRadians(0)));
    return autoPath;
  }

  private void addIntakeStep() {
    steps.add(new AutoStep.IntakePiece(drive, intake, elevator, arm));
  }

  private void addScoreStep(Pose2d scoringPose, Side robotSide) {
    steps.add(new AutoStep.Score(scoringPose, robotSide, drive, vision, intake, arm, elevator));
  }

  private void addDriveStep(Pose2d pose) {
    steps.add(new AutoStep.DriveToPose(drive, pose));
  }

  public Command get() {
    Command autoCommand = Commands.none();
    for (AutoStep step : steps) {
      autoCommand = autoCommand.andThen(step.get());
    }
    return autoCommand;
  }
}
