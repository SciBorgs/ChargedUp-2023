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
import org.sciborgs1155.lib.constants.PIDConstants;
import org.sciborgs1155.lib.constants.SystemConstants;

/** Add your docs here. */
public class MAXElbowJoint implements JointIO {
  private final CANSparkMax middleMotor;
  private final CANSparkMax leftMotor;
  private final CANSparkMax rightMotor;
  private final Encoder encoder;

  private final PIDController pid;
  private final ArmFeedforward feedforward;

  public final double offset;

  private State setpoint;

  public MAXElbowJoint(PIDConstants pidConstants, SystemConstants ffConstants, int port) {
    setpoint = new State();
    motor = Wrist.MOTOR.build(MotorType.kBrushless, port);
    offset = Math.PI;

    motor.burnFlash();

    if (newWrist) {
      pid = new PIDController(Wrist.PID.p(), Wrist.PID.i(), Wrist.PID.d());
      feedforward = new ArmFeedforward(Wrist.FF.s(), Wrist.FF.g(), Wrist.FF.v(), Wrist.FF.a());
    } else {
      pid = new PIDController(0, 0, 0);
      feedforward = new ArmFeedforward(0, 0, 0);
    }

    pid.setTolerance(0.3);
  }

  public void setPID() {}

  public void follow(ElbowIO joint) {
    this.motor.follow(joint.motor);
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

  @Override
  public void close() {}
  ;
}
