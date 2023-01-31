package org.sciborgs1155.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.robot.Constants.ElevatorConstants;
import org.sciborgs1155.robot.Constants.Motors;
import org.sciborgs1155.robot.Ports.ElevatorPorts;

public class Elevator extends SubsystemBase {
  private final CANSparkMax elevatorMotors =
      Motors.elevatorMotorConfig.buildCanSparkMaxGearbox(
          MotorType.kBrushless, ElevatorPorts.elevatorPorts);
  private final AbsoluteEncoder elevatorEncoder =
      elevatorMotors.getAbsoluteEncoder(Type.kDutyCycle);

  private final ElevatorFeedforward elevatorFeedforward =
      new ElevatorFeedforward(
          ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);
  private final ProfiledPIDController elevatorFeedback =
      new ProfiledPIDController(
          ElevatorConstants.P,
          ElevatorConstants.I,
          ElevatorConstants.D,
          ElevatorConstants.CONSTRAINTS);

  private double goal;

  public Elevator() {
    elevatorEncoder.setPositionConversionFactor(
        ElevatorConstants.GEAR_RATIO * ElevatorConstants.DISTANCEPERSPROCKETSPIN);
    elevatorEncoder.setVelocityConversionFactor(ElevatorConstants.GEAR_RATIO);
  }

  /**
   * Sets goal in meters
   *
   * @param goal
   */
  public void setGoal(double goal) {
    this.goal = goal;
  }

  @Override
  public void periodic() {
    double fb = elevatorFeedback.calculate(elevatorEncoder.getPosition(), goal);
    double ff = elevatorFeedforward.calculate(elevatorFeedback.getSetpoint().velocity);
    elevatorMotors.setVoltage(fb + ff);
  }
}
