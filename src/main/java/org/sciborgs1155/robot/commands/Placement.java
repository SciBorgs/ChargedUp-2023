package org.sciborgs1155.robot.commands;

import static org.sciborgs1155.robot.Constants.Positions.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import org.sciborgs1155.robot.subsystems.Arm;
import org.sciborgs1155.robot.subsystems.Elevator;
import org.sciborgs1155.robot.util.placement.PlacementCache;
import org.sciborgs1155.robot.util.placement.PlacementState;
import org.sciborgs1155.robot.util.placement.PlacementState.Side;
import org.sciborgs1155.robot.util.placement.PlacementTrajectory;
import org.sciborgs1155.robot.util.placement.PlacementTrajectory.Parameters;

/** Placement command factories */
public final class Placement {

  private final Arm arm;
  private final Elevator elevator;

  private final Map<Integer, PlacementTrajectory> trajectories = new HashMap<>();

  public Placement(Arm arm, Elevator elevator) {
    this.arm = arm;
    this.elevator = elevator;
    for (var trajectory : PlacementCache.loadTrajectories().entrySet()) {
      trajectories.put(trajectory.getKey(), trajectory.getValue());
    }
  }

  public Optional<PlacementTrajectory> findTrajectory(Parameters params) {

    int hash = params.hashCode();
    if (trajectories.get(hash) != null) return Optional.of(trajectories.get(hash));
    return Optional.empty();
  }

  public PlacementState state() {
    return PlacementState.fromRelative(
        elevator.getPosition(),
        arm.getElbowPosition().getRadians(),
        arm.getRelativeWristPosition().getRadians());
  }

  /** Runs elevator and arm to a state, which can include velocity */
  public Command toState(PlacementState state) {
    return Commands.parallel(
            elevator.followProfile(state.elevatorHeight()),
            arm.followProfile(state.elbowAngle(), state.wristAngle()))
        .withName("placement to state");
  }

  /** Runs elevator and arm between multiple states, uses {@link #toState(PlacementState)} */
  public Command toState(PlacementState... states) {
    Command cmd = Commands.none();
    for (var state : states) cmd = cmd.andThen(toState(state));
    return cmd;
  }

  public Command followTrajectory(Parameters params) {
    var foundTrajectory = findTrajectory(params);
    if (foundTrajectory.isEmpty()) return Commands.none();
    return Commands.parallel(
        elevator.followTrajectory(foundTrajectory.get().elevator()),
        arm.followTrajectory(foundTrajectory.get().elbow(), foundTrajectory.get().wrist()));
  }

  public Command safeToState(PlacementState state) {
    return new ConditionalCommand(
        toState(passOver(state.side()), state),
        toState(state),
        () -> state.side() != state().side());
  }

  public PlacementState passOver(Side side) {
    return side == Side.FRONT ? PASS_TO_FRONT : PASS_TO_BACK;
  }
}
