package org.sciborgs1155.robot.subsystems.placement;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.Dimensions;
import org.sciborgs1155.robot.Constants.Motors;
import org.sciborgs1155.robot.Robot;

public interface Wrist extends Sendable, AutoCloseable {

  public static Wrist create(int deviceId) {
    return Robot.isReal() ? new SparkWrist(deviceId) : new SimWrist();
  }

  public Rotation2d getPosition();

  public double getVelocity();

  public void setVoltage(double voltage);

  public double getCurrentDrawn();

  @Override
  public void close();

  @Override
  default void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("position", () -> getPosition().getDegrees(), null);
    builder.addDoubleProperty("velocity", this::getVelocity, null);
    builder.addDoubleProperty("current drawn", this::getCurrentDrawn, null);
  }

  public static class SparkWrist implements Wrist {

    private final CANSparkMax motor;
    private final AbsoluteEncoder encoder;

    public SparkWrist(int deviceId) {
      motor = Motors.WRIST.build(MotorType.kBrushless, deviceId);
      encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
    }

    @Override
    public Rotation2d getPosition() {
      return Rotation2d.fromRadians(encoder.getPosition());
    }

    @Override
    public double getVelocity() {
      return encoder.getVelocity();
    }

    @Override
    public void setVoltage(double voltage) {
      motor.setVoltage(voltage);
    }

    @Override
    public double getCurrentDrawn() {
      return motor.getOutputCurrent();
    }

    @Override
    public void close() {
      motor.close();
    }
  }

  public static class SimWrist implements Wrist {

    private final SingleJointedArmSim wristSim =
        new SingleJointedArmSim(
            DCMotor.getNeo550(1),
            1,
            SingleJointedArmSim.estimateMOI(Dimensions.CLAW_LENGTH, Dimensions.CLAW_MASS),
            Dimensions.CLAW_LENGTH,
            Dimensions.WRIST_MIN_ANGLE,
            Dimensions.WRIST_MAX_ANGLE,
            true);

    @Override
    public Rotation2d getPosition() {
      return Rotation2d.fromRadians(wristSim.getAngleRads());
    }

    @Override
    public double getVelocity() {
      return wristSim.getVelocityRadPerSec();
    }

    @Override
    public void setVoltage(double voltage) {
      wristSim.setInputVoltage(voltage);
      wristSim.update(Constants.RATE);
    }

    @Override
    public double getCurrentDrawn() {
      return wristSim.getCurrentDrawAmps();
    }

    @Override
    public void close() {}
  }
}
