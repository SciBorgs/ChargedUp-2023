package org.sciborgs1155.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.photonvision.PhotonCamera;
import org.sciborgs1155.robot.subsystems.Arm;

/** Trajectory for elevator and arm, without respect for time */
public final class Placement {

  private final Arm arm;
  private final PhotonCamera cam;

  public Placement(Arm arm, PhotonCamera cam) {
    this.arm = arm;
    this.cam = cam;
  }

  public Command goToCameraTarget() {
    return arm.runToGoal(
        arm.inverseRR(cam.getLatestResult().getBestTarget().getBestCameraToTarget()));
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
