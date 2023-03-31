package org.sciborgs1155.robot.commands;

import static org.sciborgs1155.robot.Constants.Positions.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import org.sciborgs1155.robot.subsystems.Arm;
import org.sciborgs1155.robot.subsystems.Elevator;
import org.sciborgs1155.robot.util.PlacementState;
import org.sciborgs1155.robot.util.PlacementState.Side;

/** Placement command factories */
public final class Placement {

  private final Arm arm;
  private final Elevator elevator;

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
            elevator.runToGoal(state.elevatorState()),
            arm.runToGoals(state.elbowState(), state.wristState()))
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
