package org.sciborgs1155.robot.subsystems;

import static org.sciborgs1155.robot.Constants.Intake.*;
import static org.sciborgs1155.robot.Ports.Intake.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.robot.Constants.Motors;

public class Intake extends SubsystemBase {

  private final CANSparkMax wheels;

  public Intake() {
    wheels = Motors.INTAKE.build(MotorType.kBrushless, WHEEL_MOTOR);
  }

  public Command start() {
    return runOnce(() -> wheels.set(WHEEL_SPEED));
  }

  public Command stop() {
    return runOnce(wheels::stopMotor);
  }

  public Command run() {
    return startEnd(() -> wheels.set(WHEEL_SPEED), wheels::stopMotor);
  }
}
