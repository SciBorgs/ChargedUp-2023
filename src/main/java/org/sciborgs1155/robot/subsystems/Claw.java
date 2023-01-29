package org.sciborgs1155.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.robot.Constants.Motors;
import org.sciborgs1155.robot.Constants.PlacementConstants.Intake;
import org.sciborgs1155.robot.Constants.PlacementConstants.Wrist;
import org.sciborgs1155.robot.Ports.ClawPorts;

public class Claw extends SubsystemBase {
  private CANSparkMax wheels =
      Motors.INTAKE.buildCanSparkMax(MotorType.kBrushless, ClawPorts.CLAW_WHEELS);
  private CANSparkMax wrist =
      Motors.INTAKE.buildCanSparkMax(MotorType.kBrushless, ClawPorts.CLAW_WRIST);
  private final PIDController ClawPID = new PIDController(Wrist.kP, Wrist.kI, Wrist.kD);
  private RelativeEncoder wristEncoder;

  public Claw() {
    // Enter Encoder Type
    wristEncoder = wrist.getEncoder();
  }

  public void turnOnWheels() {
    wheels.set(Intake.WHEEL_SPEED);
  }

  public void stopWheels() {
    wheels.set(0);
  }

  public void DisableWrist() {
    wrist.set(0);
  }

  public Command runWheels() {
    return this.startEnd(() -> this.turnOnWheels(), () -> this.stopWheels());
  }

  @Override
  public void periodic() {
    // Check Encoder Output
    wrist.set(ClawPID.calculate(wristEncoder.getPosition()));
  }
}
