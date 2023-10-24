package org.sciborgs1155.robot.subsystems.arm;

import static org.sciborgs1155.robot.Constants.Elbow.*;
import static org.sciborgs1155.robot.Ports.Elbow.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Encoder;
import io.github.oblarg.oblog.annotations.Log;
import java.util.List;
import org.sciborgs1155.lib.BetterArmFeedforward;
import org.sciborgs1155.lib.constants.SparkUtils;
import org.sciborgs1155.lib.failure.FaultBuilder;
import org.sciborgs1155.lib.failure.HardwareFault;
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

  private State lastSetpoint;
  private double lastVoltage;

  public RealElbow(JointConfig config) {
    middleMotor =
        SparkUtils.create(
            MIDDLE_MOTOR,
            MotorType.kBrushless,
            spark -> {
              spark.restoreFactoryDefaults();
              spark.setCANTimeout(50);
              spark.setIdleMode(IdleMode.kBrake);
              spark.setOpenLoopRampRate(0);
              spark.setSmartCurrentLimit(50);
            });
    leftMotor =
        SparkUtils.create(
            LEFT_MOTOR,
            MotorType.kBrushless,
            spark -> {
              spark.restoreFactoryDefaults();
              spark.setCANTimeout(50);
              spark.setIdleMode(IdleMode.kBrake);
              spark.setOpenLoopRampRate(0);
              spark.setSmartCurrentLimit(50);
              spark.follow(middleMotor);
            });
    rightMotor =
        SparkUtils.create(
            RIGHT_MOTOR,
            MotorType.kBrushless,
            spark -> {
              spark.restoreFactoryDefaults();
              spark.setCANTimeout(50);
              spark.setIdleMode(IdleMode.kBrake);
              spark.setOpenLoopRampRate(0);
              spark.setSmartCurrentLimit(50);
              spark.follow(middleMotor);
            });

    SparkUtils.disableFrames(middleMotor, 4, 5, 6);
    SparkUtils.disableFrames(leftMotor, 1, 2, 3, 4, 5, 6);
    SparkUtils.disableFrames(rightMotor, 1, 2, 3, 4, 5, 6);

    encoder = new Encoder(ENCODER[0], ENCODER[1]);
    encoder.setDistancePerPulse(CONVERSION);
    encoder.setReverseDirection(true);

    minAngle = config.minAngle();
    maxAngle = config.maxAngle();

    ff = config.ff().createFeedforward();
    pid = config.pid().createPIDController();
    pid.setTolerance(0.3);

    lastSetpoint = new State(getRelativeAngle().getRadians(), 0);
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
    double clampedPosition = MathUtil.clamp(setpoint.position, minAngle, maxAngle);

    double feedforward =
        ff.calculate(
            clampedPosition + getBaseAngle().getRadians(),
            lastSetpoint.velocity,
            setpoint.velocity,
            Constants.PERIOD);
    double feedback = pid.calculate(getRelativeAngle().getRadians(), clampedPosition);

    lastVoltage = feedback + feedforward;
    middleMotor.setVoltage(lastVoltage);

    lastSetpoint = setpoint;
  }

  @Override
  public void stopMoving() {
    lastVoltage = 0;
    middleMotor.stopMotor();
  }

  public State getDesiredState() {
    return lastSetpoint;
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
  public double getVoltage() {
    return lastVoltage;
  }

  @Override
  public List<HardwareFault> getFaults() {
    return FaultBuilder.create()
        .register("elbow left spark", leftMotor)
        .register("elbow middle spark", middleMotor)
        .register("elbow right spark", rightMotor)
        .register(
            "elbow encoder",
            encoder.getDistance() == 0 // no position reading
                && encoder.getRate() == 0 // no velocity reading
                && lastSetpoint.position != OFFSET // elbow is not going to 0
                && middleMotor.getAppliedOutput() != 0) // elbow is trying to move;
        .build();
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
