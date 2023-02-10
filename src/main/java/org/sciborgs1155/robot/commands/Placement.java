package org.sciborgs1155.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import org.sciborgs1155.robot.Constants.Arm.Wrist;
import org.sciborgs1155.robot.subsystems.Arm;
import org.sciborgs1155.robot.subsystems.Elevator;

/** Trajectory for elevator and arm, without respect for time */
public final class Placement {

  public static class State {
    public final double elevatorHeight;
    public final Rotation2d elbowAngle;
    public final Rotation2d wristAngle;

    public Arm.Side getSide() {
      return elbowAngle.getCos() > 0 ? Arm.Side.FRONT : Arm.Side.BACK;
    }

    public State(double elevatorHeight, Rotation2d elbowAngle, Rotation2d wristAngle) {
      this.elevatorHeight = elevatorHeight;
      this.elbowAngle = elbowAngle;
      this.wristAngle = wristAngle;
    }

    
  }
  
  public static Command goToState(Arm arm, Elevator elevator, State state) {



 
    double lowHeight =0 ;//placeholder garbage
    
    return Commands.parallel(

            /*
            Drives elevator down to lowHeight if state side is opposite to arm side
            


            */
            Commands.either(elevator.setGoal(lowHeight),Commands.none(),()->state.getSide()!=arm.getSide()).andThen(elevator.setGoal(state.elevatorHeight)),
            
            arm.setElbowGoal(state.elbowAngle),
            arm.setAbsoluteWristGoal(state.wristAngle))
        .andThen(
            Commands.waitUntil(() -> elevator.atGoal() && arm.atElbowGoal() && arm.atWrsitGoal())
            
            );
  }

  public static Command goToState(Arm arm, Elevator elevator, State... states) {
    Command cmd = Commands.none();
    for (State state : states) {
      cmd = cmd.andThen(goToState(arm, elevator, state));
    }
    return cmd;
  }
}
