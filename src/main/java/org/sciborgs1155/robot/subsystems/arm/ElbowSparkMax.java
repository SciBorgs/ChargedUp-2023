// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.subsystems.arm;

import static org.sciborgs1155.robot.Constants.Arm.*;
import static org.sciborgs1155.robot.Ports.Arm.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Encoder;
import io.github.oblarg.oblog.annotations.Log;
import org.sciborgs1155.lib.constants.PIDConstants;
import org.sciborgs1155.lib.constants.SystemConstants;

/** Add your docs here. */
public class ElbowSparkMax implements JointIO {
  private final CANSparkMax middleMotor;
  private final CANSparkMax leftMotor;
  private final CANSparkMax rightMotor;

  private final Encoder encoder;

  private final PIDController pid;
  private final ArmFeedforward ff;

  private Rotation2d baseAngle;
  private double zeroOffset;

  private State setpoint;

  public ElbowSparkMax(PIDConstants pidConstants, SystemConstants ffConstants) {

    middleMotor = Wrist.MOTOR.build(MotorType.kBrushless, MIDDLE_ELBOW_MOTOR);
    leftMotor = Wrist.MOTOR.build(MotorType.kBrushless, LEFT_ELBOW_MOTOR);
    rightMotor = Wrist.MOTOR.build(MotorType.kBrushless, RIGHT_ELBOW_MOTOR);

    encoder = new Encoder(ELBOW_ENCODER[0], ELBOW_ENCODER[1]);

    pid = new PIDController(pidConstants.p(), pidConstants.i(), pidConstants.d());
    ff = new ArmFeedforward(ffConstants.s(), ffConstants.g(), ffConstants.v(), ffConstants.a());

    setpoint = new State(getRotation().getRadians(), 0);

    baseAngle = new Rotation2d();

    middleMotor.burnFlash();
    leftMotor.burnFlash();
    rightMotor.burnFlash();

    leftMotor.follow(middleMotor);
    rightMotor.follow(middleMotor);

    encoder.setDistancePerPulse(Elbow.CONVERSION.factor());

    pid.setTolerance(0.3);
  }

  /** Elbow position relative to the chassis */
  @Log(name = "elbow position", methodName = "getRadians")
  public Rotation2d getRotation() {
    return Rotation2d.fromRadians(encoder.getDistance() + zeroOffset);
  }

  /** Sets, THEN moves to a desired state */
  public void updateDesiredState(State desiredState) {
    setpoint = desiredState;
    double feedback = pid.calculate(getRotation().getRadians(), desiredState.position);
    double feedforward = ff.calculate(desiredState.position, desiredState.velocity, 0);
    middleMotor.setVoltage(feedback + feedforward);
  }

  public State getDesiredState() {
    return setpoint;
  }

  /** Non-functional */
  public void setBaseAngle(Rotation2d baseAngle) {
    throw new UnsupportedOperationException();
  }

  @Override
  public void close() {}
  ;
}
