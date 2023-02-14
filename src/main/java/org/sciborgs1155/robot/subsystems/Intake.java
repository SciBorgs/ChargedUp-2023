package org.sciborgs1155.robot.subsystems;

import static org.sciborgs1155.robot.Constants.Intake.*;
import static org.sciborgs1155.robot.Ports.Intake.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.sciborgs1155.robot.Constants.Motors;

public class Intake extends SubsystemBase implements Loggable {
  public static class GamePieces {
    public static final Color NOTHING = new Color(0, 0, 0);
    public static final Color CONE = new Color(255, 214, 7);
    public static final Color CUBE = new Color(128, 0, 128);
  }

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 sensor = new ColorSensorV3(i2cPort);
  private final ColorMatch matcher = new ColorMatch();

  public Intake() {
    matcher.addColorMatch(GamePieces.CONE);
    matcher.addColorMatch(GamePieces.CUBE);
  }

  public Color getGamePiece() {
    Color detected = sensor.getColor();
    ColorMatchResult match = matcher.matchClosestColor(detected);

    if (match.color == GamePieces.CONE || match.color == GamePieces.CUBE) {
      return match.color;
    } else {
      return GamePieces.NOTHING;
    }
  }

  @Log(name = "applied output", methodName = "getAppliedOutput")
  private final CANSparkMax wheels = Motors.INTAKE.build(MotorType.kBrushless, WHEEL_MOTOR);

  @Log(name = "velocity", methodName = "getVelocity")
  private final RelativeEncoder encoder = wheels.getEncoder();

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
