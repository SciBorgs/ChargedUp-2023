package org.sciborgs1155.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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
import org.sciborgs1155.lib.MotorConfig;

public class Elevator extends SubsystemBase {
  private final CANSparkMax motor = Motors.ELEVATOR.buildCanSparkMaxGearbox(MotorType.kBrushless, ElevatorPorts.ELEVATOR_PORTS);
  private final RelativeEncoder encoder = motor.getEncoder(); 

  private final ElevatorFeedforward ff = new ElevatorFeedforward(Constants., 0, 0, 0);
  private final ProfiledPIDController pid =
      new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));
  // goal height
  private double height = 0;

  private double lastSpeed = 0;
  private double lastTime = Timer.getFPGATimestamp();

  private double acceleration = 0.0;

  // digital input
  DigitalInput beambreak = new DigitalInput(ElevatorPorts.BEAM_BREAK_PORTS[0]);
  DigitalInput beambreakTwo = new DigitalInput(ElevatorPorts.BEAM_BREAK_PORTS[1]);

  @Override
  public void periodic() {
    if (!beambreak.get() || !beambreakTwo.get()) {
      acceleration =
          (pid.getSetpoint().velocity - lastSpeed)
              / (Timer.getFPGATimestamp() - lastTime);
      double pidOutput = pid.calculate(encoder.getPosition(), height);
      double ffOutput = ff.calculate(pid.getSetpoint().velocity, acceleration);
      motor.setVoltage(pidOutput + ffOutput);

      lastSpeed = pid.getSetpoint().velocity;
    } else {
      motor.stopMotor();
    }
    lastTime = Timer.getFPGATimestamp();
  }

  public void setTargetHeight(double newHeight) {
    height = newHeight;
  }
}