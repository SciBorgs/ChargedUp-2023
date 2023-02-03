package org.sciborgs1155.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.Motors;
import org.sciborgs1155.robot.Ports.ClawPorts;

public class Intake extends SubsystemBase {

  /*
   * I removed the leftover wrist code and renamed to intake, now it's literally just a motor
   * simplemotorsubsystem fr
   */

  private CANSparkMax wheels;

  public Intake() {
    wheels = Motors.INTAKE.buildCanSparkMax(MotorType.kBrushless, ClawPorts.CLAW_WHEELS);
  }

  public void turnOnWheels() {
    wheels.set(Constants.Intake.WHEEL_SPEED);
  }

  public void stopWheels() {
    wheels.set(0);
  }

  /*
   * amazing
   */
  public Command runWheels() {
    return this.startEnd(() -> this.turnOnWheels(), () -> this.stopWheels());
  }
}
