package org.sciborgs1155.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.ArmConstants;
import org.sciborgs1155.robot.Constants.Motors;
import org.sciborgs1155.robot.Ports.ArmPorts;
import org.sciborgs1155.robot.Ports.ClawPorts;

public class Arm extends SubsystemBase {

  private CANSparkMax wrist =
      Motors.WRIST.buildCanSparkMax(MotorType.kBrushless, ClawPorts.CLAW_WRIST);
  private final PIDController wristFeedback =
      new PIDController(Constants.Wrist.kp, Constants.Wrist.ki, Constants.Wrist.kd);
  private final RelativeEncoder wristEncoder = wrist.getEncoder();

  private final CANSparkMax armMotors =
      Motors.ARM.buildCanSparkMaxGearbox(MotorType.kBrushless, ArmPorts.armPorts);
  private final MotorControllerGroup armGroup = new MotorControllerGroup(armMotors);
  private final RelativeEncoder armEncoder = armMotors.getEncoder();
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

  public void DisableWrist() {
    wrist.set(0);
  }

  @Override
  public void periodic() {
    // Check Encoder Output
    wrist.set(wristFeedback.calculate(wristEncoder.getPosition()));
  }
}
