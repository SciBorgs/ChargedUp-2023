package org.sciborgs1155.robot.subsystems.arm;

import static org.sciborgs1155.robot.Constants.Elevator.*;
import static org.sciborgs1155.robot.Ports.Elevator.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Encoder;
import java.util.List;
import org.sciborgs1155.lib.BetterElevatorFeedforward;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.failure.FaultBuilder;
import org.sciborgs1155.lib.failure.HardwareFault;
import org.sciborgs1155.robot.Constants;

public class RealElevator implements ElevatorIO {

  private final CANSparkMax lead;
  private final CANSparkMax left;
  private final CANSparkMax right;

  private final Encoder encoder;

  private final BetterElevatorFeedforward ff;
  private final PIDController pid;

  private final double minHeight;
  private final double maxHeight;

  private State lastSetpoint;
  private double lastVoltage;

  public RealElevator(ElevatorConfig config) {
    lead = SparkUtils.create(RIGHT_MOTOR, MotorType.kBrushless, MOTOR_CFG);
    left =
        SparkUtils.create(
            LEFT_MOTOR,
            MotorType.kBrushless,
            spark -> {
              spark.restoreFactoryDefaults();
              spark.setCANTimeout(50);
              spark.setIdleMode(IdleMode.kBrake);
              spark.setOpenLoopRampRate(0);
              spark.setSmartCurrentLimit(35);
              spark.follow(lead);
            });
    right =
        SparkUtils.create(
            MIDDLE_MOTOR,
            MotorType.kBrushless,
            spark -> {
              spark.restoreFactoryDefaults();
              spark.setCANTimeout(50);
              spark.setIdleMode(IdleMode.kBrake);
              spark.setOpenLoopRampRate(0);
              spark.setSmartCurrentLimit(35);
              spark.follow(lead);
            });

    SparkUtils.disableFrames(lead, 4, 5, 6);
    SparkUtils.disableFrames(left, 1, 2, 3, 4, 5, 6);
    SparkUtils.disableFrames(right, 1, 2, 3, 4, 5, 6);

    encoder = new Encoder(ENCODER[0], ENCODER[1]);
    encoder.setDistancePerPulse(CONVERSION_RELATIVE);
    encoder.setReverseDirection(true);

    minHeight = config.minHeight();
    maxHeight = config.maxHeight();

    ff = config.ff().createFeedforward();
    pid = config.pid().createPIDController();

    lastSetpoint = new State(getHeight(), 0);
  }

  public double getHeight() {
    return encoder.getDistance() + ZERO_OFFSET;
  }

  @Override
  public State getCurrentState() {
    return new State(getHeight(), encoder.getRate());
  }

  @Override
  public State getDesiredState() {
    return lastSetpoint;
  }

  @Override
  public void updateSetpoint(State setpoint) {
    double clampedPosition = MathUtil.clamp(setpoint.position, minHeight, maxHeight);

    double ffOutput = ff.calculate(lastSetpoint.velocity, setpoint.velocity, Constants.PERIOD);
    double fbOutput = pid.calculate(getHeight(), clampedPosition);

    lastVoltage = ffOutput + fbOutput;
    lead.setVoltage(lastVoltage);

    lastSetpoint = setpoint;
  }

  @Override
  public void stopMoving() {
    lastVoltage = 0;
    lead.stopMotor();
  }

  @Override
  public double getVoltage() {
    return lastVoltage;
  }

  @Override
  public List<HardwareFault> getFaults() {
    return FaultBuilder.create()
        .register("elevator lead spark", lead)
        .register("elevator left spark", left)
        .register("elevator right spark", right)
        .register(
            "elbow encoder",
            encoder.getDistance() == 0 // no position reading
                && encoder.getRate() == 0 // no velocity reading
                && lastSetpoint.position != ZERO_OFFSET // elbow is not going to 0
                && lead.getAppliedOutput() != 0) // elbow is trying to move;
        .build();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    ElevatorIO.super.initSendable(builder);
    pid.initSendable(builder);
    encoder.initSendable(builder);
  }

  @Override
  public void close() throws Exception {
    lead.close();
    right.close();
    left.close();
  }
}
