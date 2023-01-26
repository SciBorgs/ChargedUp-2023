package org.sciborgs1155.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import org.sciborgs1155.robot.Constants.ElevatorConstants;
import org.sciborgs1155.robot.Constants.Motors;
import org.sciborgs1155.robot.Ports.ElevatorPorts;

public class ElevatorSubsystem {
  private final CANSparkMax[] elevatorMotors = Motors.elevatorMotorConfig.buildCanSparkMax(
   MotorType.kBrushless,
   ElevatorPorts.elevatorPorts);
  private final MotorControllerGroup elevatorGroup = new MotorControllerGroup(elevatorMotors);
  private final RelativeEncoder elevatorEncoder = elevatorMotors[0].getEncoder();
  
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

  public ElevatorSubsystem() {
    elevatorEncoder.setPositionConversionFactor(ElevatorConstants.GEAR_RATIO * ElevatorConstants.MOVEMENTPERSPIN);
    elevatorEncoder.setVelocityConversionFactor(ElevatorConstants.GEAR_RATIO);
  }

 
  public void setElevatorPositon(double position){
    double fb = elevatorFeedback.calculate(elevatorEncoder.getPosition(), position);
    double ff = elevatorFeedforward.calculate(elevatorFeedback.getSetpoint().velocity);
    elevatorGroup.setVoltage(fb+ff);
  }
}
