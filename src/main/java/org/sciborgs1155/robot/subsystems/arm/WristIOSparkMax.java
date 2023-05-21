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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import org.sciborgs1155.lib.BetterArmFeedforward;
import org.sciborgs1155.lib.constants.PIDConstants;
import org.sciborgs1155.lib.constants.SystemConstants;
import org.sciborgs1155.robot.Constants;

/** Add your docs here. */
public class WristIOSparkMax implements JointIO {
  private final CANSparkMax motor = Wrist.MOTOR.build(MotorType.kBrushless, WRIST_MOTOR);
  private final DutyCycleEncoder absolute = new DutyCycleEncoder(WRIST_ABS_ENCODER);
  private final Encoder relative =
      new Encoder(WRIST_RELATIVE_ENCODER[0], WRIST_RELATIVE_ENCODER[1]);

  private final PIDController pid;
  private final BetterArmFeedforward ff;

  private Rotation2d elbowAngle = new Rotation2d();

  private State setpoint;
  private double voltage;

  public WristIOSparkMax(PIDConstants pidConstants, SystemConstants ffConstants) {
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
  public void update(State setpoint) {
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
  public void close() throws Exception {
    motor.close();
    absolute.close();
    relative.close();
  }
}
