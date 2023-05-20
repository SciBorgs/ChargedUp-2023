// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.subsystems.arm;

import static org.sciborgs1155.robot.Constants.Arm.*;
import static org.sciborgs1155.robot.Ports.Arm.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import io.github.oblarg.oblog.annotations.Log;
import org.sciborgs1155.lib.constants.PIDConstants;
import org.sciborgs1155.lib.constants.SystemConstants;

/** Add your docs here. */
public class WristSparkMax implements JointIO {
  private final CANSparkMax motor;
  private final AbsoluteEncoder encoder;

  private final PIDController pid;
  private final ArmFeedforward ff;

  private double zeroOffset;
  private Rotation2d baseAngle;

  private State setpoint;

  public WristSparkMax(PIDConstants pidConstants, SystemConstants ffConstants) {
    motor = Wrist.MOTOR.build(MotorType.kBrushless, WRIST_MOTOR);
    encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);

    pid = new PIDController(pidConstants.p(), pidConstants.i(), pidConstants.d());
    ff = new ArmFeedforward(ffConstants.s(), ffConstants.g(), ffConstants.v(), ffConstants.a());

    zeroOffset = Math.PI;
    baseAngle = new Rotation2d();

    encoder.setPositionConversionFactor(Wrist.CONVERSION.factor());
    encoder.setVelocityConversionFactor(Wrist.CONVERSION.factor() / 60.0);

    // set wrist duty cycle absolute encoder frame periods to be the same as our tickrate
    // periodic frames 3 and 4 are useless to us, so to improve performance we set them to 1155 ms
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1155);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1155);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);

    motor.burnFlash();

    pid.setTolerance(0.3);

    setpoint = new State(Math.PI, 0);
  }

  /** Wrist position relative to the forearm */
  @Log(name = "relative wrist position", methodName = "getRadians")
  public Rotation2d getRotation() {
    // encoder is zeroed fully folded in, which is actually PI, so we offset by -PI
    return Rotation2d.fromRadians(encoder.getPosition() - zeroOffset);
  }

  @Log(name = "absolute wrist position", methodName = "getRadians")
  public Rotation2d getAbsoluteWristPosition() {
    return getRotation().plus(baseAngle);
  }

  /** Sets, THEN moves to a desired state */
  public void updateDesiredState(State desiredState) {
    double feedback = pid.calculate(getRotation().getRadians(), desiredState.position);
    double feedforward = ff.calculate(desiredState.position, desiredState.velocity, 0);
    motor.setVoltage(feedback + feedforward);
  }

  public State getDesiredState() {
    return setpoint;
  }

  public void setBaseAngle(Rotation2d baseAngle) {
    this.baseAngle = baseAngle;
  }

  @Override
  public void close() {}
}
