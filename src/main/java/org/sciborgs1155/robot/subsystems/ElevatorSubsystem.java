package org.sciborgs1155.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import org.sciborgs1155.robot.Constants.ElevatorConstants;
import org.sciborgs1155.robot.Constants.Motors;
import org.sciborgs1155.robot.Ports.ElevatorPorts;

public class ElevatorSubsystem {
  private final CANSparkMax elevatorRightMotor =
      Motors.elevatorMotorConfig.buildCanSparkMax(
          MotorType.kBrushless, ElevatorPorts.rightElevatorMotor);
  private final CANSparkMax elevatorLeftMotor =
      Motors.elevatorMotorConfig.buildCanSparkMax(
          MotorType.kBrushless, ElevatorPorts.leftElevatorMotor);
  private final CANSparkMax elevatorCenterMotor =
      Motors.elevatorMotorConfig.buildCanSparkMax(
          MotorType.kBrushless, ElevatorPorts.middleElevatorMotor);
  private final RelativeEncoder elevatorLeftEncoder = elevatorLeftMotor.getEncoder();
  private final RelativeEncoder elevatorRightEncoder = elevatorRightMotor.getEncoder();
  private final ElevatorFeedforward elevatorFeedforward =
      new ElevatorFeedforward(
          ElevatorConstants.kS,
          ElevatorConstants.kG,
          ElevatorConstants.kV,
          ElevatorConstants.kA);
  private final ProfiledPIDController elevatorFeedback =
      new ProfiledPIDController(
          ElevatorConstants.P,
          ElevatorConstants.I,
          ElevatorConstants.D,
          ElevatorConstants.CONSTRAINTS);

  public ElevatorSubsystem() {}
}
