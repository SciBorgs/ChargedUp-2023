package org.sciborgs1155.robot.subsystems;

import static org.sciborgs1155.robot.Constants.Intake.*;
import static org.sciborgs1155.robot.Ports.Intake.*;

import org.sciborgs1155.lib.TestableSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Intake extends TestableSubsystem implements Loggable {

  @Log(name = "applied output", methodName = "getAppliedOutput")
  @Log(name = "current", methodName = "getOutputCurrent")
  private final CANSparkMax wheels = MOTOR.build(MotorType.kBrushless, WHEEL_MOTOR);

  @Log(name = "velocity", methodName = "getVelocity")
  private final RelativeEncoder encoder = wheels.getEncoder();

  @Log private double intakeSpeed = INTAKE_SPEED;
  @Log private double outtakeSpeed = OUTTAKE_SPEED;

  public Command set(double percent) {
    return run(() -> wheels.set(MathUtil.clamp(percent, -1, 1)));
  }

  public Command intake() {
    return set(intakeSpeed).withName("intake");
  }

  public Command outtake() {
    return set(outtakeSpeed).withName("outtake");
  }

  public Command stop() {
    return runOnce(wheels::stopMotor);
  }

  public boolean isHoldingItem() {
    return wheels.getOutputCurrent() > THRESHOLD;
  }

  @Override
  public void close() {
    wheels.close();
  }
}
