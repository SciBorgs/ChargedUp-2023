package org.sciborgs1155.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.robot.Constants.Motors;
import org.sciborgs1155.robot.Ports.ElevatorPorts;

public class Elevator extends SubsystemBase {
  private final DutyCycleEncoder elevatorEncoder =
      new DutyCycleEncoder(ElevatorPorts.ELEVATOR_ENCODER_PORT);
  private final MotorControllerGroup motors =
      new MotorControllerGroup(
          Motors.ELEVATOR.buildCanSparkMax(MotorType.kBrushed, ElevatorPorts.ELEVATOR_PORTS));

  // private final PIDController cascadePid = new PIDController(0.1, 0, 0);
  private final ElevatorFeedforward ff = new ElevatorFeedforward(0, 0, 0, 0);
  private final ProfiledPIDController elevatorControlledPID =
      new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));
  // goal height
  private double height = 0;

  private double lastSpeed = 0;
  private double lastTime = Timer.getFPGATimestamp();

  private double acceleration = 0.0;

  // digital input
  DigitalInput beambreak = new DigitalInput(ElevatorPorts.BEAM_BREAK_PORT);

  @Override
  public void periodic() {
    if (!beambreak.get()) {
      acceleration =
          (elevatorControlledPID.getSetpoint().velocity - lastSpeed)
              / (Timer.getFPGATimestamp() - lastTime);
      double pidOutput = elevatorControlledPID.calculate(elevatorEncoder.getDistance(), height);
      double ffOutput = ff.calculate(elevatorControlledPID.getSetpoint().velocity, acceleration);
      motors.setVoltage(pidOutput + ffOutput);

      lastSpeed = elevatorControlledPID.getSetpoint().velocity;
      lastTime = Timer.getFPGATimestamp();
    } else {
      motors.setVoltage(0);
    }
  }

  public void setTargetHeight(double newHeight) {
    height = newHeight;
  }
}

// mid .56 metres
// high 1.06 metrses

  // public void setVelocity() {
    // elevatorOne.set(
    //     cascadePid.calculate(
    //         cascadeEncoder.getPosition())) /* + ff.calculate(velocity, 2, 0.001) */;
  // }
// }

// mid and high
// public void setMidHeight() {
//     Elevator.setTargetHeight(Constants.midHeight);
//   }

// public void setHighHeight() {
//     Elevator.setTargetHeight(Constants.highHeight);
//   }
