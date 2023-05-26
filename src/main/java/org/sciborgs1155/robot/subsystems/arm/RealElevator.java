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
import org.sciborgs1155.lib.constants.ElevatorFFConstants;
import org.sciborgs1155.lib.constants.PIDConstants;
import org.sciborgs1155.robot.Constants;

public class RealElevator implements ElevatorIO {

  private final CANSparkMax lead;
  private final CANSparkMax left;
  private final CANSparkMax right;

  private final Encoder encoder;

  private final PIDController pid;
  private final BetterElevatorFeedforward ff;

  private State setpoint;
  private double voltage;

  public RealElevator(PIDConstants pidConstants, ElevatorFFConstants ffConstants) {
    lead = MOTOR_CFG.build(MotorType.kBrushless, RIGHT_MOTOR);
    left = MOTOR_CFG.build(MotorType.kBrushless, LEFT_MOTOR);
    right = MOTOR_CFG.build(MotorType.kBrushless, MIDDLE_MOTOR);

    left.follow(lead);
    right.follow(lead);

    lead.burnFlash();
    left.burnFlash();
    right.burnFlash();

    encoder = new Encoder(ENCODER[0], ENCODER[1]);
    encoder.setDistancePerPulse(CONVERSION_RELATIVE);
    encoder.setReverseDirection(true);

    pid = pidConstants.createPIDController();
    ff = ffConstants.createFeedforward();

    setpoint = new State(getHeight(), 0);
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
  public void updateSetpoint(State setpoint) {
    double ffOutput = ff.calculate(this.setpoint.velocity, setpoint.velocity, Constants.PERIOD);
    double fbOutput = pid.calculate(getHeight(), setpoint.position);

    voltage = ffOutput + fbOutput;
    lead.setVoltage(voltage);

    this.setpoint = setpoint;
  }

  @Override
  public void stopMoving() {
    lead.stopMotor();
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
