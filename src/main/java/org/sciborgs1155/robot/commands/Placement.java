package org.sciborgs1155.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.sciborgs1155.lib.PlacementState;
import org.sciborgs1155.robot.subsystems.Arm;
import org.sciborgs1155.robot.subsystems.Elevator;

/** Placement command factories */
public class Placement {

  private final Arm arm;
  private final Elevator elevator;

  public Placement(Arm arm, Elevator elevator) {
    this.arm = arm;
    this.elevator = elevator;
  }

  public Command toState(PlacementState state) {
    return Commands.parallel(elevator.runToGoal(state.elevatorState()));
  }

  public Command toState(PlacementState... states) {
    Command cmd = Commands.none();
    for (var state : states) cmd = cmd.andThen(toState(state));
    return cmd;
  }
}
