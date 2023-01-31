package org.sciborgs1155.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.sciborgs1155.robot.Constants.Motors;
import org.sciborgs1155.robot.Constants.PlacementConstants.Elbow;
import org.sciborgs1155.robot.Ports.ArmPorts;

public class Arm extends SubsystemBase{
  private final CANSparkMax armMotors =
      Motors.ARM.buildCanSparkMaxGearbox(MotorType.kBrushless, ArmPorts.armPorts);
  private final MotorControllerGroup armGroup = new MotorControllerGroup(armMotors);
  private final RelativeEncoder armEncoder = armMotors.getEncoder();
  private final ArmFeedforward armFeedforward =
      new ArmFeedforward(Elbow.kS, Elbow.kG, Elbow.kV, Elbow.kA);
  private final ProfiledPIDController armFeedback =
      new ProfiledPIDController(Elbow.kP, Elbow.kI, Elbow.kD, Elbow.CONSTRAINTS);
  private Rotation2d goal;
  public Arm() {
    armEncoder.setPositionConversionFactor(Elbow.GEAR_RATIO * Elbow.MOVEMENTPERSPIN);
    armEncoder.setVelocityConversionFactor(Elbow.GEAR_RATIO);
  }
  public void setGoal(Rotation2d goal) {
      this.goal = goal;
  }
  @Override
  public void periodic() {
    double fb = armFeedback.calculate(
      armEncoder.getPosition(), 
      goal.getRadians());
    double ff =
        armFeedforward.calculate(
            armFeedback.getSetpoint().position, armFeedback.getSetpoint().velocity);
    armGroup.setVoltage(fb + ff);
  
  }
    
}
