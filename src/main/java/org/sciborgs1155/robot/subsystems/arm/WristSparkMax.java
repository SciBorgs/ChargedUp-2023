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
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import io.github.oblarg.oblog.annotations.Log;

import org.sciborgs1155.lib.BetterArmFeedforward;
import org.sciborgs1155.lib.constants.PIDConstants;
import org.sciborgs1155.lib.constants.SystemConstants;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.subsystems.Arm.JointIO;

/** Add your docs here. */
public class WristSparkMax implements JointIO {
  private final CANSparkMax motor = Wrist.MOTOR.build(MotorType.kBrushless, WRIST_MOTOR);
  private final DutyCycleEncoder absolute = new DutyCycleEncoder(0); // TODO
  private final Encoder relative = new Encoder(0, 1); // TODO

  private final PIDController pid;
  private final BetterArmFeedforward ff;

  private Rotation2d elbowAngle = new Rotation2d();

  private State setpoint;
  private double voltage;

  public WristSparkMax(PIDConstants pidConstants, SystemConstants ffConstants) {
    pid = pidConstants.create();
    ff = ffConstants.createArmFF();

    absolute.setDistancePerRotation(Wrist.CONVERSION.factor());
    relative.setDistancePerPulse(Wrist.CONVERSION.factor());

    // set wrist duty cycle absolute encoder frame periods to be the same as our tickrate
    // periodic frames 3 and 4 are useless to us, so to improve performance we set them to 1155 ms
    // motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1155);
    // motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1155);
    // motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    // motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);

    motor.burnFlash();

    pid.setTolerance(0.3);

    // setpoint = new State(Math.PI, 0);
    setpoint = new State(absolute.getDistance(), 0);
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

  /** Sets, THEN moves to a desired state */
  @Override
  public void updateDesiredState(State desiredState) {
    double feedforward = ff.calculate(desiredState.position + getBaseAngle().getRadians(), setpoint.velocity, desiredState.velocity, Constants.PERIOD);
    double feedback = pid.calculate(getRelativeAngle().getRadians(), desiredState.position);

    voltage = feedback + feedforward;
    motor.setVoltage(voltage);

    setpoint = desiredState;
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
  public double getVoltage() {
      // TODO Auto-generated method stub
      return 0;
  }

  @Override
  public void close() throws Exception {
      motor.close();
      absolute.close();
      relative.close();
  }
}
