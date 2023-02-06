package org.sciborgs1155.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.sciborgs1155.lib.PlacementTrajectory;
import org.sciborgs1155.robot.subsystems.Arm;
import org.sciborgs1155.robot.subsystems.Elevator;

public class Placement {

  private final Arm arm;
  private final Elevator elevator;

  public Placement(Arm arm, Elevator elevator) {
    this.arm = arm;
    this.elevator = elevator;
  }

  public boolean atGoal() {
    return elevator.atGoal() && arm.atElbowGoal() && arm.atWrsitGoal();
  }

  public Command goTo(Arm arm, Elevator elevator, PlacementTrajectory.State state) {
    return Commands.parallel(
            elevator.setGoal(state.elevatorHeight),
            arm.setElbowGoal(state.elbowAngle),
            arm.setAbsoluteWristGoal(state.wristAngle))
        .andThen(Commands.waitUntil(this::atGoal));
  }
}
