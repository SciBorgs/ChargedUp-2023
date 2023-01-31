package org.sciborgs1155.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.Motors;
import org.sciborgs1155.robot.Ports.ClawPorts;

public class Claw extends SubsystemBase {
  private CANSparkMax wheels =
      Motors.INTAKE.buildCanSparkMax(MotorType.kBrushless, ClawPorts.CLAW_WHEELS);

  public Claw() {
    // Enter Encoder Type
  }

  public void turnOnWheels() {
    wheels.set(Constants.Intake.ClawWheelsEnableSpeed);
  }

  public void stopWheels() {
    wheels.set(0);
  }

  public Command runWheels() {
    return this.startEnd(() -> this.turnOnWheels(), () -> this.stopWheels());
  }
}
