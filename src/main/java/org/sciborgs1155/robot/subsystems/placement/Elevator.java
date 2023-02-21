package org.sciborgs1155.robot.subsystems.placement;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.Arm.ElevatorConstants;
import org.sciborgs1155.robot.Constants.Dimensions;
import org.sciborgs1155.robot.Constants.Motors;
import org.sciborgs1155.robot.Robot;

public interface Elevator extends Sendable, AutoCloseable {

  public static Elevator create(int leadId, int leftId, int rightId) {
    return Robot.isReal() ? new SparkElevator(leadId, leftId, rightId) : new SimElevator();
  }

  public double getPosition();

  public double getVelocity();

  public void setVoltage(double voltage);

  public double getCurrentDrawn();

  public boolean atBottom();

  public boolean atTop();

  @Override
  public void close();

  @Override
  default void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("position", this::getPosition, null);
    builder.addDoubleProperty("velocity", this::getVelocity, null);
    builder.addDoubleProperty("current drawn", this::getCurrentDrawn, null);
    builder.addBooleanProperty("at bottom", this::atBottom, null);
    builder.addBooleanProperty("at top", this::atTop, null);
  }

  public static class SparkElevator implements Elevator {
    private final CANSparkMax lead, left, right;
    private final RelativeEncoder encoder;

    public SparkElevator(int leadId, int leftId, int rightId) {
      lead = Motors.ELEVATOR.build(MotorType.kBrushless, leadId);
      left = Motors.ELEVATOR.build(MotorType.kBrushless, leftId);
      right = Motors.ELEVATOR.build(MotorType.kBrushless, rightId);

      left.follow(lead);
      right.follow(lead);

      encoder = lead.getAlternateEncoder(Constants.THROUGH_BORE_CPR);

      encoder.setPositionConversionFactor(ElevatorConstants.ENCODER_POSITION_FACTOR);
      encoder.setVelocityConversionFactor(ElevatorConstants.ENCODER_VELOCITY_FACTOR);

      lead.burnFlash();
      left.burnFlash();
      right.burnFlash();
    }

    @Override
    public double getPosition() {
      return encoder.getPosition();
    }

    @Override
    public double getVelocity() {
      return encoder.getVelocity();
    }

    @Override
    public void setVoltage(double voltage) {
      lead.setVoltage(voltage);
    }

    @Override
    public double getCurrentDrawn() {
      return lead.getOutputCurrent();
    }

    @Override
    public boolean atBottom() {
      return getPosition() < 0.02;
    }

    @Override
    public boolean atTop() {
      return Math.abs(getPosition() - Dimensions.ELEVATOR_MAX_HEIGHT) < 0.02;
    }

    @Override
    public void close() {
      lead.close();
      left.close();
      right.close();
    }
  }

  public static class SimElevator implements Elevator {
    private final ElevatorSim sim =
        new ElevatorSim(
            DCMotor.getNEO(3),
            1 / ElevatorConstants.ENCODER_POSITION_FACTOR,
            4,
            Units.inchesToMeters(2),
            Dimensions.ELEVATOR_MIN_HEIGHT,
            Dimensions.ELEVATOR_MAX_HEIGHT,
            true);

    @Override
    public double getPosition() {
      return sim.getPositionMeters();
    }

    @Override
    public double getVelocity() {
      return sim.getVelocityMetersPerSecond();
    }

    @Override
    public void setVoltage(double voltage) {
      sim.setInputVoltage(voltage);
      sim.update(Constants.RATE);
    }

    @Override
    public double getCurrentDrawn() {
      return sim.getCurrentDrawAmps();
    }

    @Override
    public boolean atBottom() {
      return sim.wouldHitLowerLimit(getPosition());
    }

    @Override
    public boolean atTop() {
      return sim.wouldHitUpperLimit(getPosition());
    }

    @Override
    public void close() {}
  }
}
