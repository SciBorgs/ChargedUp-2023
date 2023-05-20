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

/** Add your docs here. */
public class MAXWristJoint implements JointIO {
  private final CANSparkMax motor;
  private final AbsoluteEncoder encoder;

  private final PIDController pid;
  private final ArmFeedforward feedforward;

  public final double offset;

  private State setpoint;

  public MAXWristJoint(boolean newWrist, int port) {
    setpoint = new State();
    motor = Wrist.MOTOR.build(MotorType.kBrushless, port);
    encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
    offset = Math.PI;

    motor.burnFlash();

    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1155);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1155);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);

    if (newWrist) {
      pid = new PIDController(Wrist.PID.p(), Wrist.PID.i(), Wrist.PID.d());
      feedforward = new ArmFeedforward(Wrist.FF.s(), Wrist.FF.g(), Wrist.FF.v(), Wrist.FF.a());
    } else {
      pid = new PIDController(0, 0, 0);
      feedforward = new ArmFeedforward(0, 0, 0);
    }

    pid.setTolerance(0.3);
  }

  public Rotation2d getRotation() {
    return Rotation2d.fromRadians(encoder.getPosition() - offset);
  }

  /** Sets, THEN moves to a desired state */
  public void updateDesiredState(State state) {
    this.setpoint = state;
  }

  public State getDesiredState() {
    return setpoint;
  }

  /** DOES NOTHING */
  public void follow() {}

  @Override
  public void close() {}
  ;
}
