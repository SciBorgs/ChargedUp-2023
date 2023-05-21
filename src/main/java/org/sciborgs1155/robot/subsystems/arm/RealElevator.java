package org.sciborgs1155.robot.subsystems.arm;

import static org.sciborgs1155.robot.Constants.Elevator.*;
import static org.sciborgs1155.robot.Ports.Elevator.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Encoder;
import org.sciborgs1155.lib.BetterElevatorFeedforward;
import org.sciborgs1155.robot.Constants;

public class RealElevator implements ElevatorIO {

  private final CANSparkMax lead;
  private final CANSparkMax left;
  private final CANSparkMax right;

  private final Encoder encoder;

  private final PIDController pid = PID.create();
  private final BetterElevatorFeedforward ff = FF.createElevatorFF();

  private State setpoint;
  private double voltage;

  public RealElevator() {
    lead = MOTOR.build(MotorType.kBrushless, RIGHT_MOTOR);
    left = MOTOR.build(MotorType.kBrushless, LEFT_MOTOR);
    right = MOTOR.build(MotorType.kBrushless, MIDDLE_MOTOR);

    left.follow(lead);
    right.follow(lead);

    lead.burnFlash();
    left.burnFlash();
    right.burnFlash();

    encoder = new Encoder(ENCODER[0], ENCODER[1]);
    encoder.setDistancePerPulse(RELATIVE_CONVERSION.factor());

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
  public void update(State setpoint) {
    double ffOutput = ff.calculate(this.setpoint.velocity, setpoint.velocity, Constants.PERIOD);
    double fbOutput = pid.calculate(getHeight(), setpoint.position);

    voltage = ffOutput + fbOutput;
    lead.setVoltage(voltage);

    this.setpoint = setpoint;
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
  public void initSendable(SendableBuilder builder) {
    ElevatorIO.super.initSendable(builder);
    pid.initSendable(builder);
    encoder.initSendable(builder);
  }

  @Override
  public void close() throws Exception {
    lead.close();
    right.close();
    left.close();
  }
}
