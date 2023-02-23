package org.sciborgs1155.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;
import org.sciborgs1155.lib.Vision;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.commands.Autos.ShouldBeInDiffFile.RobotSide;
import org.sciborgs1155.robot.subsystems.*;

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
    autoPath.addIntakeStep();
    autoPath.addScoreStep(Constants.Field.ScoringPoints.BLUE_EIGHT, RobotSide.FRONT);
    return autoPath;
  }

  private void addIntakeStep() {
    steps.add(new AutoStep.IntakePiece(drive, intake, elevator, arm));
  }

  private void addScoreStep(Pose2d scoringPose, RobotSide robotSide) {
    steps.add(new AutoStep.Score(scoringPose, robotSide, drive, vision, intake, arm, elevator));
  }

  public Command get() {
    Command autoCommand = Commands.none();
    for (AutoStep step : steps) {
      autoCommand = autoCommand.andThen(step.get());
    }
    return autoCommand;
  }
}
