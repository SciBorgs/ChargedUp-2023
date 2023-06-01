package org.sciborgs1155.robot.subsystems.arm;

import static org.sciborgs1155.robot.Constants.Elbow.*;
import static org.sciborgs1155.robot.Ports.Elbow.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Encoder;
import io.github.oblarg.oblog.annotations.Log;
import org.sciborgs1155.lib.BetterArmFeedforward;
import org.sciborgs1155.robot.Constants;

public class RealElbow implements JointIO {
  private final CANSparkMax middleMotor;
  private final CANSparkMax leftMotor;
  private final CANSparkMax rightMotor;

  private final Encoder encoder;

  private final BetterArmFeedforward ff;
  private final PIDController pid;

  private final double minAngle;
  private final double maxAngle;

  private State setpoint;
  private double voltage;

  public RealElbow(JointConfig config) {
    middleMotor = MOTOR_CFG.build(MotorType.kBrushless, MIDDLE_MOTOR);
    leftMotor = MOTOR_CFG.build(MotorType.kBrushless, LEFT_MOTOR);
    rightMotor = MOTOR_CFG.build(MotorType.kBrushless, RIGHT_MOTOR);

    leftMotor.follow(middleMotor);
    rightMotor.follow(middleMotor);

    middleMotor.burnFlash();
    leftMotor.burnFlash();
    rightMotor.burnFlash();

    encoder = new Encoder(ENCODER[0], ENCODER[1]);
    encoder.setDistancePerPulse(CONVERSION);
    encoder.setReverseDirection(true);

    minAngle = config.minAngle();
    maxAngle = config.maxAngle();

    ff = config.ff().createFeedforward();
    pid = config.pid().createPIDController();
    pid.setTolerance(0.3);

    setpoint = new State(getRelativeAngle().getRadians(), 0);
  }

  /** Elbow position relative to the chassis */
  @Log(name = "elbow position", methodName = "getRadians")
  public Rotation2d getRelativeAngle() {
    return Rotation2d.fromRadians(encoder.getDistance() + OFFSET);
  }

  @Override
  public State getCurrentState() {
    return new State(getRelativeAngle().getRadians(), encoder.getRate());
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
    middleMotor.setVoltage(voltage);

    this.setpoint = setpoint;
  }

  @Override
  public void stopMoving() {
    voltage = 0;
    middleMotor.stopMotor();
  }

  public State getDesiredState() {
    return setpoint;
  }

  @Override
  public void setBaseAngle(Rotation2d baseAngle) {
    throw new UnsupportedOperationException("elbow should not be offset with a base angle");
  }

  @Override
  public Rotation2d getBaseAngle() {
    return new Rotation2d();
  }

  @Override
  public boolean isFailing() {
    return encoder.getDistance() == 0 // no position reading
        && encoder.getRate() == 0 // no velocity reading
        && encoder.getDistance() != OFFSET // elbow is not going to 0
        && middleMotor.getAppliedOutput() != 0; // elbow is trying to move
  }

  @Override
  public double getVoltage() {
    return voltage;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    JointIO.super.initSendable(builder);
    pid.initSendable(builder);
    encoder.initSendable(builder);
  }

  @Override
  public void close() {}
}
