package org.sciborgs1155.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import org.sciborgs1155.robot.Constants.ArmConstants;
import org.sciborgs1155.robot.Constants.Motors;
import org.sciborgs1155.robot.Ports.ArmPorts;

public class Arm {
  private final CANSparkMax[] armMotors =
      Motors.armMotorConfig.buildCanSparkMax(MotorType.kBrushless, ArmPorts.armPorts);
  private final MotorControllerGroup armGroup = new MotorControllerGroup(armMotors);
  private final RelativeEncoder armEncoder = armMotors[0].getEncoder();
  private final ArmFeedforward armFeedforward =
      new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);
  private final ProfiledPIDController armFeedback =
      new ProfiledPIDController(
          ArmConstants.P, ArmConstants.I, ArmConstants.D, ArmConstants.CONSTRAINTS);

  public Arm() {
    armEncoder.setPositionConversionFactor(ArmConstants.GEAR_RATIO * ArmConstants.MOVEMENTPERSPIN);
    armEncoder.setVelocityConversionFactor(ArmConstants.GEAR_RATIO);
  }

  public void setArmPosition(double position) {
    double fb = armFeedback.calculate(armEncoder.getPosition(), position);
    double ff =
        armFeedforward.calculate(
            armFeedback.getSetpoint().position, armFeedback.getSetpoint().velocity);
    armGroup.setVoltage(fb + ff);
  }
}
