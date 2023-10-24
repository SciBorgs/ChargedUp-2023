package org.sciborgs1155.robot.subsystems.arm;

import static org.sciborgs1155.robot.Ports.Wrist.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import java.util.List;
import org.sciborgs1155.lib.BetterArmFeedforward;
import org.sciborgs1155.lib.constants.MotorConfig;
import org.sciborgs1155.lib.failure.FaultBuilder;
import org.sciborgs1155.lib.failure.HardwareFault;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.subsystems.arm.ArmConstants.Wrist;  

public class RealWrist implements JointIO {

  private final CANSparkMax motor;
  private final DutyCycleEncoder absolute;
  private final Encoder relative;

  private final BetterArmFeedforward ff;
  private final PIDController pid;

  private final double minAngle;
  private final double maxAngle;

  private Rotation2d elbowAngle = new Rotation2d();

  private State lastSetpoint;
  private double lastVoltage;

  public RealWrist(JointConfig config) {
    motor = Wrist.MOTOR_CFG.build(MotorType.kBrushless, MOTOR);
    absolute = new DutyCycleEncoder(ABS_ENCODER);
    relative = new Encoder(RELATIVE_ENCODER[0], RELATIVE_ENCODER[1]);

    absolute.setPositionOffset(Wrist.ZERO_OFFSET);
    absolute.setDistancePerRotation(Wrist.CONVERSION_ABS);
    relative.setDistancePerPulse(Wrist.CONVERSION_RELATIVE);
    relative.setReverseDirection(true);

    MotorConfig.disableFrames(motor, 4, 5, 6);

    motor.burnFlash();

    ff = config.ff().createFeedforward();
    pid = config.pid().createPIDController();
    pid.setTolerance(0.3);

    minAngle = config.minAngle();
    maxAngle = config.maxAngle();

    lastSetpoint = new State(2, 0);
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
    return lastSetpoint;
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
    motor.setVoltage(lastVoltage);

    lastSetpoint = setpoint;
  }

  @Override
  public void stopMoving() {
    lastVoltage = 0;
    motor.stopMotor();
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
    return lastVoltage;
  }

  @Override
  public List<HardwareFault> getFaults() {
    return FaultBuilder.create()
        .register("wrist spark", motor)
        .register("wrist encoder", absolute)
        .build();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    JointIO.super.initSendable(builder);
    pid.initSendable(builder);
    builder.addDoubleProperty("absolute duty cycle", absolute::getAbsolutePosition, null);
    builder.addDoubleProperty("relative encoder position", relative::getDistance, null);
  }

  @Override
  public void close() throws Exception {
    motor.close();
    absolute.close();
    relative.close();
  }
}
