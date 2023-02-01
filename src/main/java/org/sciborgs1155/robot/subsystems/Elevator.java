package org.sciborgs1155.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.robot.Constants.Motors;
import org.sciborgs1155.robot.Ports.ElevatorPorts;

public class Elevator extends SubsystemBase {

  // this is all good
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;

  private final ElevatorFeedforward ff;
  private final ProfiledPIDController pid;

  // digital input
  /*
   * the DigitalInputs should be private and final, since they are sensors that must be encapsulated within the subsystem
   */
  DigitalInput beambreak;
  DigitalInput beambreakTwo;

  // goal height
  private double height = 0;

  private double lastSpeed = 0;
  private double lastTime = Timer.getFPGATimestamp();

  private double acceleration = 0.0;

  public Elevator() {
    motor =
        Motors.ELEVATOR.buildCanSparkMaxGearbox(MotorType.kBrushless, ElevatorPorts.ELEVATOR_PORTS);
    encoder = motor.getEncoder();

    ff = new ElevatorFeedforward(0, 0, 0, 0);
    pid = new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));

    beambreak = new DigitalInput(ElevatorPorts.BEAM_BREAK_PORTS[0]);
    beambreakTwo = new DigitalInput(ElevatorPorts.BEAM_BREAK_PORTS[1]);
  }

  public void setTargetHeight(double newHeight) {
    height = newHeight;
  }

  @Override
  public void periodic() {
    if (!beambreak.get() || !beambreakTwo.get()) {
      // i'll add a derivative stream soon to make these calculations cleaner
      acceleration =
          (pid.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
      double pidOutput = pid.calculate(encoder.getPosition(), height);
      double ffOutput = ff.calculate(pid.getSetpoint().velocity, acceleration);
      motor.setVoltage(pidOutput + ffOutput);

      lastSpeed = pid.getSetpoint().velocity;
    } else {
      motor.stopMotor();
    }
    lastTime = Timer.getFPGATimestamp();
  }
}
