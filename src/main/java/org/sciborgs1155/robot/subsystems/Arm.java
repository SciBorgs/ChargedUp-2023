package org.sciborgs1155.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.robot.Constants.ArmConstants;
import org.sciborgs1155.robot.Constants.Motors;
import org.sciborgs1155.robot.Ports.ArmPorts;

public class Arm extends SubsystemBase {
  private final CANSparkMax armMotors =
      Motors.armMotorConfig.buildCanSparkMaxGearbox(MotorType.kBrushless, ArmPorts.armPorts);
  private final RelativeEncoder armEncoder = armMotors.getEncoder();
  private final ArmFeedforward armFeedforward =
      new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);
  private final ProfiledPIDController armFeedback =
      new ProfiledPIDController(
          ArmConstants.P, ArmConstants.I, ArmConstants.D, ArmConstants.CONSTRAINTS);
  private Rotation2d goal;

  public Arm() {
    armEncoder.setPositionConversionFactor(ArmConstants.GEAR_RATIO * ArmConstants.MOVEMENTPERSPIN);
    armEncoder.setVelocityConversionFactor(ArmConstants.GEAR_RATIO);
  }

  public void setGoal(Rotation2d rotation) {
    this.goal = rotation;
  }

  @Override
  public void periodic() {
    double fb = armFeedback.calculate(armEncoder.getPosition(), goal.getRadians());
    double ff =
        armFeedforward.calculate(
            armFeedback.getSetpoint().position, armFeedback.getSetpoint().velocity);
    armMotors.setVoltage(fb + ff);
  }
}
