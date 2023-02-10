package org.sciborgs1155.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;

/** Trajectory for elevator and arm, without respect for time */
public final class Placement {

  public static class State {
    public final double elevatorHeight;
    public final Rotation2d elbowAngle;
    public final Rotation2d wristAngle;

    public State(double elevatorHeight, Rotation2d elbowAngle, Rotation2d wristAngle) {
      this.elevatorHeight = elevatorHeight;
      this.elbowAngle = elbowAngle;
      this.wristAngle = wristAngle;
    }
  }

  // public static Command goToState(Arm arm, Elevator elevator, State state) {
  //   return Commands.parallel(
  //           elevator.setGoal(state.elevatorHeight),
  //           arm.setElbowGoal(state.elbowAngle),
  //           arm.setAbsoluteWristGoal(state.wristAngle))
  //       .andThen(
  //           Commands.waitUntil(() -> elevator.atGoal() && arm.atElbowGoal() &&
  // arm.atWrsitGoal()));
  // }

  // public static Command goToState(Arm arm, Elevator elevator, State... states) {
  //   Command cmd = Commands.none();
  //   for (State state : states) {
  //     cmd = cmd.andThen(goToState(arm, elevator, state));
  //   }
  //   return cmd;
  // }
}
