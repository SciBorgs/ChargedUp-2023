package org.sciborgs1155.robot.subsystems.placement;

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
  public static Elbow create(int leadId, int leftId, int rightId) {
    return Robot.isReal() ? new SparkElbow(leadId, leftId, rightId) : new SimElbow();
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

  public static class SparkElbow implements Elbow {
    private final CANSparkMax lead, left, right;
    private final RelativeEncoder encoder;

    public SparkElbow(int leadId, int leftId, int rightId) {
      lead = Motors.ELBOW.build(MotorType.kBrushless, leadId);
      left = Motors.ELBOW.build(MotorType.kBrushless, leftId);
      right = Motors.ELBOW.build(MotorType.kBrushless, rightId);

      left.follow(lead);
      right.follow(lead);

      encoder = lead.getAlternateEncoder(Constants.THROUGH_BORE_CPR);

      encoder.setPositionConversionFactor(ElbowConstants.ENCODER_POSITION_FACTOR);
      encoder.setVelocityConversionFactor(ElbowConstants.ENCODER_VELOCITY_FACTOR);

      lead.burnFlash();
      left.burnFlash();
      right.burnFlash();
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
      left.close();
      right.close();
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
