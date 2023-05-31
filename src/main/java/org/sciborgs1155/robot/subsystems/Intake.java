package org.sciborgs1155.robot.subsystems;

import static org.sciborgs1155.robot.Constants.Intake.*;
import static org.sciborgs1155.robot.Ports.Intake.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.sciborgs1155.robot.subsystems.arm.ArmState.GamePiece;

public class Intake extends SubsystemBase implements Loggable, AutoCloseable {

  @Log(name = "applied output", methodName = "getAppliedOutput")
  @Log(name = "current", methodName = "getOutputCurrent")
  private final CANSparkMax wheels = MOTOR_CFG.build(MotorType.kBrushless, WHEEL_MOTOR);

  @Log(name = "velocity", methodName = "getVelocity")
  private final RelativeEncoder encoder = wheels.getEncoder();

  @Log private double coneSpeed = CONE_SPEED;
  @Log private double cubeSpeed = CUBE_SPEED;

  public Command set(double percent) {
    return run(() -> wheels.set(MathUtil.clamp(percent, -1, 1)))
        .finallyDo(end -> wheels.stopMotor());
  }

  public Command intake(GamePiece gamePiece) {
    return set(gamePiece == GamePiece.CONE ? coneSpeed : cubeSpeed).withName("intaking");
  }

  public Command outtake(GamePiece gamePiece) {
    return set(-1 * (gamePiece == GamePiece.CONE ? coneSpeed : cubeSpeed)).withName("outtaking");
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
