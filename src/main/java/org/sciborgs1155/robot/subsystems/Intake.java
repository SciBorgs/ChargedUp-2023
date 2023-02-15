package org.sciborgs1155.robot.subsystems;

import static org.sciborgs1155.robot.Constants.Intake.*;
import static org.sciborgs1155.robot.Ports.Intake.*;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import org.sciborgs1155.lib.CustomPeriodRunnables;

public class Intake extends SubsystemBase implements Loggable {
  public enum GamePieces {
    NOTHING(new Color(0, 0, 0)),
    CONE(new Color(83, 133, 33)),
    CUBE(new Color(60, 103, 93));

    public final Color color;

    private GamePieces(Color color) {
      this.color = color;
    }
  }

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 sensor = new ColorSensorV3(i2cPort);
  private final ColorMatch matcher = new ColorMatch();

  private boolean led = true;

  // 8.5
  // 3 - 2

  private final double MIN_R_CONE = 91;
  private final double MAX_R_CONE = 108;
  private final double MIN_G_CONE = 113;
  private final double MAX_G_CONE = 125;
  private final double MIN_B_CONE = 28;
  private final double MAX_B_CONE = 45;

  private final double LED_MIN_R_CONE = 79;
  private final double LED_MAX_R_CONE = 91;
  private final double LED_MIN_G_CONE = 120;
  private final double LED_MAX_G_CONE = 133;
  private final double LED_MIN_B_CONE = 23;
  private final double LED_MAX_B_CONE = 51;

  private final double MIN_R_CUBE = 71;
  private final double MAX_R_CUBE = 81;
  private final double MIN_G_CUBE = 115;
  private final double MAX_G_CUBE = 125;
  private final double MIN_B_CUBE = 54;
  private final double MAX_B_CUBE = 65;

  private final double LED_MIN_R_CUBE = 65;
  private final double LED_MAX_R_CUBE = 70;
  private final double LED_MIN_G_CUBE = 115;
  private final double LED_MAX_G_CUBE = 125;
  private final double LED_MIN_B_CUBE = 65;
  private final double LED_MAX_B_CUBE = 70;

  public Intake() {
    matcher.addColorMatch(GamePieces.CONE.color);
    matcher.addColorMatch(GamePieces.CUBE.color);

    CustomPeriodRunnables.add(this::test, 0.5);
  }

  public GamePieces getGamePiece() {
    Color detected = sensor.getColor();

    double r = detected.red * 255;
    double g = detected.green * 255;
    double b = detected.blue * 255;

    System.out.println(r + " " + g + " " + b);

    if (led) {
      if (r >= LED_MIN_R_CONE
          && r <= LED_MAX_R_CONE
          && g >= LED_MIN_G_CONE
          && g <= LED_MAX_G_CONE
          && b >= LED_MIN_B_CONE
          && b <= LED_MAX_B_CONE) {
        return GamePieces.CONE;
      } else if (r >= LED_MIN_R_CUBE
          && r <= LED_MAX_R_CUBE
          && g >= LED_MIN_G_CUBE
          && g <= LED_MAX_G_CUBE
          && b >= LED_MIN_B_CUBE
          && b <= LED_MAX_B_CUBE) {
        return GamePieces.CUBE;
      } else {
        return GamePieces.NOTHING;
      }
    } else {
      if (r >= MIN_R_CONE
          && r <= MAX_R_CONE
          && g >= MIN_G_CONE
          && g <= MAX_G_CONE
          && b >= MIN_B_CONE
          && b <= MAX_B_CONE) {
        return GamePieces.CONE;
      } else if (r >= MIN_R_CUBE
          && r <= MAX_R_CUBE
          && g >= MIN_G_CUBE
          && g <= MAX_G_CUBE
          && b >= MIN_B_CUBE
          && b <= MAX_B_CUBE) {
        return GamePieces.CUBE;
      } else {
        return GamePieces.NOTHING;
      }
    }
  }

  public void test() {
    GamePieces piece = getGamePiece();
    System.out.println(piece.name());
  }

  // @Log(name = "applied output", methodName = "getAppliedOutput")
  // private final CANSparkMax wheels = Motors.INTAKE.build(MotorType.kBrushless, WHEEL_MOTOR);

  // @Log(name = "velocity", methodName = "getVelocity")
  // private final RelativeEncoder encoder = wheels.getEncoder();

  // public Command start() {
  //   return runOnce(() -> wheels.set(WHEEL_SPEED));
  // }

  // public Command stop() {
  //   return runOnce(wheels::stopMotor);
  // }

  // public Command run() {
  //   return startEnd(() -> wheels.set(WHEEL_SPEED), wheels::stopMotor);
  // }
}
