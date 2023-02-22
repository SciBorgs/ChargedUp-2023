package org.sciborgs1155.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.sciborgs1155.lib.Derivative;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.Arm.Elbow;
import org.sciborgs1155.robot.Constants.Arm.Wrist;
import org.sciborgs1155.robot.Constants.Dimensions;
import org.sciborgs1155.robot.Constants.Motors;
import org.sciborgs1155.robot.Ports;

public class Arm extends SubsystemBase implements Loggable, AutoCloseable {

  private final ArmFeedforward wristFeedforward =
      new ArmFeedforward(Elbow.kS, Elbow.kG, Elbow.kV, Elbow.kA);

  private final ArmFeedforward elbowFeedForward =
      new ArmFeedforward(Elbow.kS, Elbow.kG, Elbow.kV, Elbow.kA);

  @Log(name = "Elbow Feedback")
  private final ProfiledPIDController elbowFeedback =
      new ProfiledPIDController(Elbow.kP, Elbow.kI, Elbow.kD, Elbow.CONSTRAINTS);

  @Log(name = "Wrist Feedback")
  private final ProfiledPIDController wristFeedback =
      new ProfiledPIDController(Wrist.kP, Wrist.kI, Wrist.kD, Wrist.CONSTRAINTS);

  @Log(name = "Wrist Acceleration", methodName = "getLastOutput")
  private final Derivative wristAccel = new Derivative();

  @Log(name = "Elbow Acceleration", methodName = "getLastOutput")
  private final Derivative elbowAccel = new Derivative();

  private final CANSparkMax elbowLead, elbowLeft, elbowRight, wrist;
  private final RelativeEncoder elbowEncoder;
  private final AbsoluteEncoder wristEncoder;

  public Arm() {
    this.elbowLead = Motors.ELBOW.build(MotorType.kBrushless, Ports.Arm.MIDDLE_ELBOW_MOTOR);
    this.elbowLeft = Motors.ELBOW.build(MotorType.kBrushless, Ports.Arm.LEFT_ELBOW_MOTOR);
    this.elbowRight = Motors.ELBOW.build(MotorType.kBrushless, Ports.Arm.RIGHT_ELBOW_MOTOR);
    this.wrist = Motors.WRIST.build(MotorType.kBrushless, Ports.Arm.WRIST_MOTOR);

    elbowLeft.follow(elbowLead);
    elbowRight.follow(elbowLead);

    elbowEncoder = elbowLead.getAlternateEncoder(Constants.THROUGH_BORE_CPR);
    wristEncoder = wrist.getAbsoluteEncoder(Type.kDutyCycle);

    elbowEncoder.setPositionConversionFactor(Elbow.ENCODER_POSITION_FACTOR);
    elbowEncoder.setVelocityConversionFactor(Elbow.ENCODER_VELOCITY_FACTOR);

    elbowLead.burnFlash();
    elbowLeft.burnFlash();
    elbowRight.burnFlash();
  }

  public Rotation2d getElbowPosition() {
    return Rotation2d.fromRadians(elbowEncoder.getPosition());
  }

  /** Wrist position relative to the forearm */
  public Rotation2d getRelativeWristPosition() {
    return Rotation2d.fromRadians(wristEncoder.getPosition());
  }

  /** Wrist position relative to chassis */
  @Log(name = "wrist absolute positon", methodName = "getDegrees")
  public Rotation2d getAbsoluteWristPosition() {
    return getRelativeWristPosition().plus(getElbowPosition());
  }

  /** Elbow goal relative to the chassis */
  public Rotation2d getElbowGoal() {
    return Rotation2d.fromRadians(elbowFeedback.getGoal().position);
  }

  /** Wrist goal relative to forearm */
  public Rotation2d getRelativeWristGoal() {
    return Rotation2d.fromRadians(wristFeedback.getGoal().position);
  }

  /** Wrist goal relative to the chassis */
  public Rotation2d getAbsoluteWristGoal() {
    return getRelativeWristGoal().plus(getElbowGoal());
  }
    
  /** Sets elbow goal relative to the chassis */
  public Command setElbowGoal(TrapezoidProfile.State goal) {
    return runOnce(
        () -> elbowFeedback.setGoal(MathUtil.clamp(goal.position, Dimensions.ELBOW_MIN_ANGLE, Dimensions.ELBOW_MAX_ANGLE))
    );
  }

  /** Sets wrist goal relative to the forearm */
  public Command setWristGoal(TrapezoidProfile.State goal) {
    return runOnce(
        () -> wristFeedback.setGoal(MathUtil.clamp(goal.position, Dimensions.WRIST_MIN_ANGLE, Dimensions.WRIST_MAX_ANGLE))
    );
  }

  public Command setGoals(TrapezoidProfile.State elbowGoal, TrapezoidProfile.State wristGoal) {
    return setElbowGoal(elbowGoal).andThen(setWristGoal(wristGoal));
  }

      /** Runs elbow to goal relative to the chassis */
      public Command runElbowToGoal(TrapezoidProfile.State goal) {
        return setElbowGoal(goal).andThen(Commands.waitUntil(elbowFeedback::atGoal));
    }

    /** Runs wrist to goal relative to the forearm */
    public Command runWristToGoal(TrapezoidProfile.State goal) {
        return setWristGoal(goal).andThen(Commands.waitUntil(wristFeedback::atGoal));
    }

    /** Runs elbow and wrist go provided goals, with the wrist goal relative to the forearm */
    public Command runToGoals(TrapezoidProfile.State elbowGoal, TrapezoidProfile.State wristGoal) {
        return setGoals(elbowGoal, wristGoal)
                .andThen(Commands.waitUntil(() -> elbowFeedback.atGoal() && wristFeedback.atGoal()));
    }

  @Override
  public void periodic() {
    double elbowFB = elbowFeedback.calculate(elbowEncoder.getPosition());
    double elbowFF =
        elbowFeedForward.calculate(
            elbowFeedback.getSetpoint().position,
            elbowFeedback.getSetpoint().velocity,
            elbowAccel.calculate(elbowFeedback.getSetpoint().velocity));
    elbowLead.setVoltage(elbowFB + elbowFF);

    // wrist feedback is calculated using an absolute angle setpoint, rather than a
    // relative one
    // this means the extra voltage calculated to cancel out gravity is kG * cos(θ +
    // ϕ), where θ is
    // the elbow setpoint and ϕ is the wrist setpoint
    // the elbow angle is used as a setpoint instead of current position because
    // we're using a
    // profiled pid controller, which means setpoints are achievable states, rather
    // than goals
    double wristFB = wristFeedback.calculate(wristEncoder.getPosition());
    double wristFF =
        wristFeedforward.calculate(
            wristFeedback.getSetpoint().position + elbowFeedback.getSetpoint().position,
            wristFeedback.getSetpoint().velocity,
            wristAccel.calculate(wristFeedback.getSetpoint().velocity));
    wrist.setVoltage(wristFB + wristFF);
  }

  @Override
  public void close() {
    elbowLead.close();
    elbowLeft.close();
    elbowRight.close();
    wrist.close();
  }
}
