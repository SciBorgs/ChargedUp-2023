package org.sciborgs1155.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Arrays;
import org.photonvision.PhotonCamera;
import org.sciborgs1155.lib.PlacementState;
import org.sciborgs1155.robot.subsystems.Arm;
import org.sciborgs1155.robot.subsystems.Elevator;

/** Trajectory for elevator and arm, without respect for time */
public final class Placement {

    public static Command goToCameraTarget(Arm arm, Elevator elevator, PhotonCamera cam) {
        return Commands.either(
                goToState(
                        arm,
                        elevator,
                        PlacementState.fromIK(
                                cam.getLatestResult().getBestTarget().getBestCameraToTarget().getTranslation(),
                                elevator.getHeight())),
                Commands.none(),
                () -> cam.getLatestResult().hasTargets());
    }

    // TODO (andrew): safe goToState command
    // 1. sets wrist to 0 and runs elevator to set height
    // 2. runs to goal state

    public static Command goToState(Arm arm, Elevator elevator, PlacementState state) {
        return Commands.parallel(
                elevator.runToGoal(state.elevatorHeight()),
                arm.runToGoals(state.elbowAngle(), state.wristAngle()));
    }

    public static Command goToState(Arm arm, Elevator elevator, PlacementState... states) {
        return Arrays.stream(states)
                .map(state -> goToState(arm, elevator, state))
                .reduce(Commands.none(), Command::andThen);
    }
}
