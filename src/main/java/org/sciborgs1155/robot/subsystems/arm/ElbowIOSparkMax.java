// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.subsystems.arm;

import static org.sciborgs1155.robot.Constants.Arm.*;
import static org.sciborgs1155.robot.Ports.Arm.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Encoder;
import io.github.oblarg.oblog.annotations.Log;
import org.sciborgs1155.lib.BetterArmFeedforward;
import org.sciborgs1155.lib.constants.PIDConstants;
import org.sciborgs1155.lib.constants.SystemConstants;
import org.sciborgs1155.robot.Constants;

/** Add your docs here. */
public class ElbowIOSparkMax implements JointIO {
  private final CANSparkMax middleMotor;
  private final CANSparkMax leftMotor;
  private final CANSparkMax rightMotor;

  private final Encoder encoder;

  private final PIDController pid;
  private final BetterArmFeedforward ff;

  private State setpoint;
  private double voltage;

  public ElbowIOSparkMax(PIDConstants pidConstants, SystemConstants ffConstants) {

    middleMotor = Wrist.MOTOR.build(MotorType.kBrushless, MIDDLE_ELBOW_MOTOR);
    leftMotor = Wrist.MOTOR.build(MotorType.kBrushless, LEFT_ELBOW_MOTOR);
    rightMotor = Wrist.MOTOR.build(MotorType.kBrushless, RIGHT_ELBOW_MOTOR);

    encoder = new Encoder(ELBOW_ENCODER[0], ELBOW_ENCODER[1]);

    this.pid = pidConstants.create();
    this.ff = ffConstants.createArmFF();

    leftMotor.follow(middleMotor);
    rightMotor.follow(middleMotor);

    encoder.setDistancePerPulse(Elbow.CONVERSION.factor());

    middleMotor.burnFlash();
    leftMotor.burnFlash();
    rightMotor.burnFlash();

    setpoint = new State(getRelativeAngle().getRadians(), 0);

    pid.setTolerance(0.3);
  }

  /** Elbow position relative to the chassis */
  @Log(name = "elbow position", methodName = "getRadians")
  public Rotation2d getRelativeAngle() {
    return Rotation2d.fromRadians(encoder.getDistance() + Elbow.OFFSET);
  }

  @Override
  public State getCurrentState() {
    return new State(getRelativeAngle().getRadians(), encoder.getRate());
  }

  /** Sets, AND moves to a desired state */
  @Override
  public void update(State setpoint) {
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
        && encoder.getDistance() != Elbow.OFFSET // elbow is not going to 0
        && middleMotor.getAppliedOutput() != 0; // elbow is trying to move
  }

  @Override
  public double getVoltage() {
    return voltage;
  }

  @Override
  public void close() {}
}
