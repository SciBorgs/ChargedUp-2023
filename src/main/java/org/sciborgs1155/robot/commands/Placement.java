package org.sciborgs1155.robot.commands;

/** Trajectory for elevator and arm, without respect for time */
public final class Placement {

  // public Command goToCameraTarget() {
  //   return Commands.either(
  //       arm.runToGoals(
  //           ArmState.fromIK(
  //               cam.getLatestResult().getBestTarget().getBestCameraToTarget().getTranslation())),
  //       Commands.none(),
  //       () -> cam.getLatestResult().hasTargets());
  // }

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
