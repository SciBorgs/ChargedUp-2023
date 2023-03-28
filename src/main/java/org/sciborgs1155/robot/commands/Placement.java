package org.sciborgs1155.robot.commands;

import static org.sciborgs1155.robot.Constants.Positions.*;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import java.util.HashMap;
import java.util.Map;
import org.sciborgs1155.robot.subsystems.Arm;
import org.sciborgs1155.robot.subsystems.Elevator;
import org.sciborgs1155.robot.util.placement.PlacementState;
import org.sciborgs1155.robot.util.placement.PlacementState.Side;
import org.sciborgs1155.robot.util.placement.PlacementTrajectory;

/** Placement command factories */
public final class Placement {

  private final Arm arm;
  private final Elevator elevator;

  private final Map<Integer, PlacementTrajectory> trajectories = new HashMap<>();

  public Placement(Arm arm, Elevator elevator) {
    this.arm = arm;
    this.elevator = elevator;
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
            elevator.runToGoal(new State(state.elevatorHeight(), 0)),
            arm.runToGoals(
                new State(state.elbowAngle().getRadians(), 0),
                new State(state.wristAngle().getRadians(), 0)))
        .withName("placement to state");
  }

  /** Runs elevator and arm between multiple states, uses {@link #toState(PlacementState)} */
  public Command toState(PlacementState... states) {
    Command cmd = Commands.none();
    for (var state : states) cmd = cmd.andThen(toState(state));
    return cmd;
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
