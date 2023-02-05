package org.sciborgs1155.robot.subsystems;

import static org.sciborgs1155.robot.Constants.Arm.*;
import static org.sciborgs1155.robot.Ports.Arm.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import org.sciborgs1155.lib.Derivative;
import org.sciborgs1155.lib.Visualizer;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.Dimensions;
import org.sciborgs1155.robot.Constants.Motors;

public class Arm extends SubsystemBase implements Loggable {

  private final CANSparkMax wrist;
  private final RelativeEncoder wristEncoder;

  private final ProfiledPIDController wristFeedback;
  private final ArmFeedforward wristFeedforward;

  private final CANSparkMax elbowLead, elbowFollow;
  private final RelativeEncoder elbowEncoder;

  private final ArmFeedforward elbowFeedforward;
  private final ProfiledPIDController elbowFeedback;

  private Derivative elbowAccel = new Derivative();

  private Derivative wristAccel = new Derivative();

  private Rotation2d elbowGoal = new Rotation2d();
  private Rotation2d wristGoal = new Rotation2d();

  // simulation
  private final SingleJointedArmSim elbowSim;
  private final SingleJointedArmSim wristSim;

  public Arm() {
    wrist = Motors.WRIST.build(MotorType.kBrushless, WRIST_MOTOR);
    wristEncoder = wrist.getEncoder();

    wristFeedback = new ProfiledPIDController(Wrist.kP, Wrist.kI, Wrist.kD, Wrist.CONSTRAINTS);
    wristFeedforward = new ArmFeedforward(Wrist.kS, Wrist.kG, Wrist.kV, Wrist.kA);

    elbowLead = Motors.ELBOW.build(MotorType.kBrushless, LEFT_ELBOW_MOTOR);
    elbowFollow = Motors.ELBOW.build(MotorType.kBrushless, RIGHT_ELBOW_MOTOR);
    elbowEncoder = elbowLead.getEncoder();
    elbowFollow.follow(elbowLead);

    elbowFeedforward = new ArmFeedforward(Elbow.kS, Elbow.kG, Elbow.kV, Elbow.kA);
    elbowFeedback = new ProfiledPIDController(Elbow.kP, Elbow.kI, Elbow.kD, Elbow.CONSTRAINTS);

    elbowEncoder.setPositionConversionFactor(Elbow.GEAR_RATIO * Elbow.MOVEMENT_PER_SPIN);
    elbowEncoder.setVelocityConversionFactor(Elbow.GEAR_RATIO);

    elbowSim =
        new SingleJointedArmSim(
            DCMotor.getNEO(2),
            1,
            1,
            Dimensions.FOREARM_LENGTH,
            Dimensions.ELBOW_MIN_ANGLE,
            Dimensions.ELBOW_MAX_ANGLE,
            1,
            true);

    wristSim =
        new SingleJointedArmSim(
            DCMotor.getNeo550(1),
            1,
            1,
            Dimensions.CLAW_LENGTH,
            Dimensions.WRIST_MIN_ANGLE,
            Dimensions.WRIST_MAX_ANGLE,
            1,
            true);
  }

  public Rotation2d getElbowPosition() {
    return Rotation2d.fromRadians(elbowEncoder.getPosition());
  }

  public Rotation2d getWristPosition() {
    return Rotation2d.fromRadians(wristEncoder.getPosition());
  }

  public Rotation2d getElbowGoal() {
    return elbowGoal;
  }

  public Rotation2d getWristGoal() {
    return wristGoal;
  }

  public Command setElbowGoal(Rotation2d elbowGoal) {
    return runOnce(() -> this.elbowGoal = elbowGoal);
  }

  public Command setWristGoal(Rotation2d wristGoal) {
    return runOnce(() -> this.wristGoal = wristGoal);
  }

  @Override
  public void periodic() {

    double elbowfb = elbowFeedback.calculate(elbowEncoder.getPosition(), elbowGoal.getRadians());
    double elbowff =
        elbowFeedforward.calculate(
            elbowFeedback.getSetpoint().position,
            elbowFeedback.getSetpoint().velocity,
            elbowAccel.calculate(elbowFeedback.getSetpoint().velocity));
    elbowLead.setVoltage(elbowfb + elbowff);

    double wristfb = wristFeedback.calculate(wristEncoder.getPosition(), wristGoal.getRadians());
    double wristff =
        wristFeedforward.calculate(
            wristFeedback.getSetpoint().position,
            wristFeedback.getSetpoint().velocity,
            wristAccel.calculate(wristFeedback.getSetpoint().velocity));
    wrist.setVoltage(wristfb + wristff);

    Visualizer.getInstance().setArmPositions(getElbowPosition(), getWristPosition());
  }

  @Override
  public void simulationPeriodic() {
    elbowSim.setInputVoltage(elbowLead.getAppliedOutput());
    elbowSim.update(Constants.RATE);
    elbowEncoder.setPosition(elbowSim.getAngleRads());

    wristSim.setInputVoltage(wrist.getAppliedOutput());
    wristSim.update(Constants.RATE);
    wristEncoder.setPosition(wristSim.getAngleRads());
  }
}
