package org.sciborgs1155.robot.subsystems.arm;

import static org.sciborgs1155.robot.Constants.Elevator.*;
import static org.sciborgs1155.robot.Ports.Elevator.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Encoder;
import org.sciborgs1155.lib.BetterElevatorFeedforward;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.subsystems.Arm.ElevatorIO;

public class ElevatorSparkMax implements ElevatorIO {

  private final CANSparkMax lead = MOTOR.build(MotorType.kBrushless, RIGHT_MOTOR);
  private final CANSparkMax left = MOTOR.build(MotorType.kBrushless, LEFT_MOTOR);
  private final CANSparkMax right = MOTOR.build(MotorType.kBrushless, MIDDLE_MOTOR);

  private final Encoder encoder = new Encoder(ENCODER[0], ENCODER[1], true); // TODO ?

  private final PIDController pid = PID.create();
  private final BetterElevatorFeedforward ff = FF.createElevatorFF();

  private State setpoint;
  private double voltage;

  public ElevatorSparkMax() {
    left.follow(lead);
    right.follow(lead);

    encoder.setDistancePerPulse(RELATIVE_CONVERSION.factor());

    lead.burnFlash();
    left.burnFlash();
    right.burnFlash();

    setpoint = new State();
  }

  public double getHeight() {
    return encoder.getDistance() + ZERO_OFFSET;
  }

  @Override
  public State getState() {
    return new State(getHeight(), encoder.getRate());
  }

  @Override
  public State getDesiredState() {
    return setpoint;
  }

  @Override
  public void updateDesiredState(State state) {
    double ffOutput = ff.calculate(setpoint.velocity, state.velocity, Constants.PERIOD);
    double fbOutput = pid.calculate(getHeight(), state.position);

    voltage = ffOutput + fbOutput;
    lead.setVoltage(voltage);

    setpoint = state;
  }

  @Override
  public boolean isFailing() {
    return false;
  }

  @Override
  public double getVoltage() {
    return voltage;
  }

  @Override
  public void close() throws Exception {
    lead.close();
    right.close();
    left.close();
  }
}
