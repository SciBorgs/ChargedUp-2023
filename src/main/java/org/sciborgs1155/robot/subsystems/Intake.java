package org.sciborgs1155.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.robot.Constants.Motors;
import org.sciborgs1155.robot.Constants.PlacementConstants;
import org.sciborgs1155.robot.Ports.ClawPorts;

public class Intake extends SubsystemBase {

  private final CANSparkMax wheels;

  public Intake() {
    wheels = Motors.INTAKE.build(MotorType.kBrushless, ClawPorts.CLAW_WHEELS);
  }

  public Command start() {
    return runOnce(() -> wheels.set(PlacementConstants.Intake.WHEEL_SPEED));
  }

  public Command stop() {
    return runOnce(wheels::stopMotor);
  }

  public Command run() {
    return startEnd(() -> wheels.set(PlacementConstants.Intake.WHEEL_SPEED), wheels::stopMotor);
  }
}
