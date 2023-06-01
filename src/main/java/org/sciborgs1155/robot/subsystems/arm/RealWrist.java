package org.sciborgs1155.robot.subsystems.arm;

import static org.sciborgs1155.robot.Constants.Wrist.*;
import static org.sciborgs1155.robot.Ports.Wrist.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import org.sciborgs1155.lib.BetterArmFeedforward;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.subsystems.arm.JointIO.JointConfig;

public class RealWrist implements JointIO {

  private final CANSparkMax motor;
  private final DutyCycleEncoder absolute;
  private final Encoder relative;

  private final BetterArmFeedforward ff;
  private final PIDController pid;

  private Rotation2d elbowAngle = new Rotation2d();

  private State setpoint;
  private double voltage;

  public RealWrist(JointConfig config) {
    motor = MOTOR_CFG.build(MotorType.kBrushless, MOTOR);
    absolute = new DutyCycleEncoder(ABS_ENCODER);
    relative = new Encoder(RELATIVE_ENCODER[0], RELATIVE_ENCODER[1]);

    absolute.setPositionOffset(ZERO_OFFSET);
    absolute.setDistancePerRotation(CONVERSION_ABS);
    relative.setDistancePerPulse(CONVERSION_RELATIVE);
    relative.setReverseDirection(true);

    motor.burnFlash();

    ff = config.ff().createFeedforward();
    pid = config.pid().createPIDController();

    pid.setTolerance(0.3);

    setpoint = new State(2, 0);
  }

  @Override
  public Rotation2d getRelativeAngle() {
    return Rotation2d.fromRadians(absolute.getDistance());
  }

  @Override
  public State getCurrentState() {
    return new State(getRelativeAngle().getRadians(), relative.getRate());
  }

  @Override
  public State getDesiredState() {
    return setpoint;
  }

  @Override
  public void updateSetpoint(State setpoint) {
    double feedforward =
        ff.calculate(
            setpoint.position + getBaseAngle().getRadians(),
            this.setpoint.velocity,
            setpoint.velocity,
            Constants.PERIOD);
    double feedback = pid.calculate(getRelativeAngle().getRadians(), setpoint.position);

    voltage = feedback + feedforward;
    motor.setVoltage(voltage);

    this.setpoint = setpoint;
  }

  @Override
  public void stopMoving() {
    voltage = 0;
    motor.stopMotor();
  }

  @Override
  public void setBaseAngle(Rotation2d baseAngle) {
    elbowAngle = baseAngle;
  }

  @Override
  public Rotation2d getBaseAngle() {
    return elbowAngle;
  }

  @Override
  public boolean isFailing() {
    return absolute.isConnected();
    // relative.getDistance() == 0
    //     && relative.getVelocity() == 0
    //     && relative.position() != 0;
  }

  @Override
  public double getVoltage() {
    return voltage;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    JointIO.super.initSendable(builder);
    pid.initSendable(builder);
    builder.addDoubleProperty("absolute duty cycle", absolute::getAbsolutePosition, null);
    builder.addDoubleProperty("relative encoder position", relative::getDistance, null);
  }

  @Override
  public void close() throws Exception {
    motor.close();
    absolute.close();
    relative.close();
  }
}
