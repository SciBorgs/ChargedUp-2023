package org.sciborgs1155.robot.subsystems.joints;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.Arm.ElbowConstants;
import org.sciborgs1155.robot.Constants.Dimensions;
import org.sciborgs1155.robot.Constants.Motors;
import org.sciborgs1155.robot.Robot;

public interface Elbow extends Sendable, AutoCloseable {
  public static Elbow create(int leadDeviceId, int followDeviceId) {
    return Robot.isReal() ? new SparkElbow(leadDeviceId, followDeviceId) : new SimElbow();
  }

  public Rotation2d getPosition();

  public double getVelocity();

  public void setVoltage(double voltage);

  public double getCurrentDrawn();

  @Override
  default void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("position", () -> getPosition().getDegrees(), null);
    builder.addDoubleProperty("velocity", this::getVelocity, null);
    builder.addDoubleProperty("current drawn", this::getCurrentDrawn, null);
  }

  public static class SparkElbow implements Elbow {
    private final CANSparkMax lead, follow;
    private final RelativeEncoder encoder;

    public SparkElbow(int leadDeviceId, int followDeviceId) {
      lead = Motors.ELBOW.build(MotorType.kBrushless, leadDeviceId);
      follow = Motors.ELBOW.build(MotorType.kBrushless, followDeviceId);

      encoder = lead.getAlternateEncoder(Constants.THROUGH_BORE_CPR);

      follow.follow(lead);

      encoder.setPositionConversionFactor(
          ElbowConstants.GEAR_RATIO * ElbowConstants.MOVEMENT_PER_SPIN);
      encoder.setVelocityConversionFactor(ElbowConstants.GEAR_RATIO);
    }

    @Override
    public Rotation2d getPosition() {
      return Rotation2d.fromRadians(encoder.getPosition());
    }

    @Override
    public double getVelocity() {
      return encoder.getVelocity();
    }

    public void setVoltage(double voltage) {
      lead.setVoltage(voltage);
    }

    @Override
    public double getCurrentDrawn() {
      return lead.getOutputCurrent();
    }

    @Override
    public void close() {
      lead.close();
      follow.close();
    }
  }

  public static class SimElbow implements Elbow {

    private final SingleJointedArmSim elbowSim =
        new SingleJointedArmSim(
            DCMotor.getNEO(2),
            ElbowConstants.GEAR_RATIO,
            SingleJointedArmSim.estimateMOI(Dimensions.FOREARM_LENGTH, 2),
            Dimensions.FOREARM_LENGTH,
            Dimensions.ELBOW_MIN_ANGLE,
            Dimensions.ELBOW_MAX_ANGLE,
            true);

    @Override
    public Rotation2d getPosition() {
      return Rotation2d.fromRadians(elbowSim.getAngleRads());
    }

    @Override
    public double getVelocity() {
      return elbowSim.getVelocityRadPerSec();
    }

    @Override
    public void setVoltage(double voltage) {
      elbowSim.setInputVoltage(voltage);
      elbowSim.update(Constants.RATE);
    }

    @Override
    public double getCurrentDrawn() {
      return elbowSim.getCurrentDrawAmps();
    }

    @Override
    public void close() {}
  }
}
