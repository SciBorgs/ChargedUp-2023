package org.sciborgs1155.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.photonvision.PhotonCamera;
import org.sciborgs1155.robot.subsystems.Arm;
import org.sciborgs1155.robot.subsystems.Elevator;
import org.sciborgs1155.robot.util.State;

/** Trajectory for elevator and arm, without respect for time */
public final class Placement {

  public static Command goToCameraTarget(Arm arm, Elevator elevator, PhotonCamera cam) {
    return Commands.either(
        goToState(
            arm,
            elevator,
            State.fromIK(
                cam.getLatestResult().getBestTarget().getBestCameraToTarget().getTranslation(),
                elevator.getHeight())),
        Commands.none(),
        () -> cam.getLatestResult().hasTargets());
  }

  // TODO (andrew): safe goToState command
  // 1. sets wrist to 0 and runs elevator to set height
  // 2. runs to goal state

  /** Runs arm and elevator to setpoints, specified in a {@link org.sciborgs1155.robot.util.State} */
  public static Command goToState(Arm arm, Elevator elevator, State state) {
    return Commands.parallel(
        elevator.runToGoal(state.elevatorHeight()),
        arm.runToGoals(state.elbowAngle(), state.wristAngle()));
  }

  public static Command goToState(Arm arm, Elevator elevator, State... states) {
    Command cmd = Commands.none();
    for (State state : states) {
      cmd = cmd.andThen(goToState(arm, elevator, state));
    }
    return cmd;
  }
}
