package org.sciborgs1155.robot.subsystems.arm;

import static org.sciborgs1155.robot.Constants.Elevator.*;
import static org.sciborgs1155.robot.Ports.Elevator.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Encoder;
import org.sciborgs1155.robot.subsystems.Arm.Elevator;

public class ElevatorSparkMax implements Elevator {

  private State setpoint;

  private boolean isfailing;

  private final Encoder encoder = new Encoder(ENCODER[0], ENCODER[1], true);
  private final double offset = ZERO_OFFSET;

  private CANSparkMax lead = MOTOR.build(MotorType.kBrushless, RIGHT_MOTOR);

  private CANSparkMax left = MOTOR.build(MotorType.kBrushless, LEFT_MOTOR);
  private CANSparkMax right = MOTOR.build(MotorType.kBrushless, MIDDLE_MOTOR);

  private final PIDController pid = new PIDController(PID.p(), PID.i(), PID.d());
  private final ElevatorFeedforward ff = new ElevatorFeedforward(FF.s(), FF.g(), FF.v(), FF.a());

  public ElevatorSparkMax(boolean isfailing) {
    this.isfailing = isfailing;

    left.follow(lead);
    right.follow(lead);
    encoder.setDistancePerPulse(RELATIVE_CONVERSION.factor());

    lead.burnFlash();
    left.burnFlash();
    right.burnFlash();

    setpoint = new State();
  }

  public double getHeight() {
    return encoder.getDistance() + offset;
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
    setpoint = state;

    double fbOutput = pid.calculate(getHeight(), setpoint.position);
    double ffOutput = ff.calculate(setpoint.velocity);

    lead.setVoltage(ffOutput + fbOutput);
  }

  @Override
  public boolean isFailing() {
    return isfailing;
  }

  @Override
  public void close() throws Exception {
    lead.close();
    right.close();
    left.close();
  }
}
