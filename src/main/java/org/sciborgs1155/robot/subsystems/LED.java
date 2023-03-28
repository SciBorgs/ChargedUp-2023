package org.sciborgs1155.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.robot.Constants.ledConst;
import org.sciborgs1155.robot.Ports.ledPorts;
import org.sciborgs1155.robot.commands.Scoring.GamePiece;

public class LED extends SubsystemBase {

  private static AddressableLED led1;
  // private static AddressableLED led2;
  private static AddressableLEDBuffer led1Buffer;
  // private static AddressableLEDBuffer led2Buffer;
  static double time = 0.0;

  public enum LEDColors {
    RAINBOW,
    CUBE,
    CONE,
    AUTO
  }

  public LED() {
    led1 = new AddressableLED(ledPorts.led1);
    led1Buffer = new AddressableLEDBuffer(ledConst.buffer1Length);
    led1.setLength(led1Buffer.getLength());
    led1.setData(led1Buffer);
    led1.start();

    // led2 = new AddressableLED(Ports.LED.led2);
    // led2Buffer = new AddressableLEDBuffer(Constants.led.led2Buffer);
    // led2.setLength(Constants.led.led2Length);
    // led2.start();
  }

  public Command setGamePieceColor(GamePiece gamePiece) {
    if (gamePiece == GamePiece.CUBE) {
      return Commands.run(() -> GamePieceColors(GamePiece.CUBE), this);
    } else if (gamePiece == GamePiece.CONE) {
      return Commands.run(() -> GamePieceColors(GamePiece.CONE), this);
    } else {
      // LEDColor matches no standard LEDColor enum
      return Commands.run(() -> errorLED(), this);
    }
  }

  public Command setPatterns(LEDColors desiredColor) {
    if ( desiredColor == LEDColors.RAINBOW) {
      return Commands.run(() -> LedPatterns(LEDColors.RAINBOW), this);
    }
    else {
      return Commands.run(() -> errorLED(), this);
    }
  }

  // note: is there a reason that you're using setRGB now instead of setLED?
  // from looking at the source code, it seems like if setRGB works, setLED should work too

  public void GamePieceColors(GamePiece gamePiece) {
    if (gamePiece == GamePiece.CONE) {
      for (int i = 0; i < led1Buffer.getLength(); i++) {
        led1Buffer.setLED(i, Color.kDarkOrange);
        // led2Buffer.setLED(i, Color.kYellow);
      }
      led1.setData(led1Buffer);

    } else if (gamePiece == GamePiece.CUBE) {
      for (int i = 0; i < led1Buffer.getLength(); i++) {
        led1Buffer.setLED(i, Color.kPurple);
        // led2Buffer.setLED(i, Color.kBlue);
      }
      led1.setData(led1Buffer);
    } 
  }

  public void LedPatterns(LEDColors ledColor) {
    if (ledColor == LEDColors.RAINBOW) {
      time += .005;
      for (int i = 0; i < led1Buffer.getLength(); i++) {

        final double constant = i / (led1Buffer.getLength() * (Math.PI / 2));
        double green = Math.sin(time + (constant));
        double blue = Math.cos(time + (constant));
        double red = -Math.sin(time + (constant));

        green *= 255 / 2;
        blue *= 255 / 2;
        red *= 255 / 2;

        green += 255 / 2;
        blue += 255 / 2;
        red += 255 / 2;

        led1Buffer.setRGB(i, (int) red, (int) green, (int) blue);
        led1.setData(led1Buffer);
      } // for loop
    } // while loop
  }

  public void errorLED() {
    for (int i = 0; i < led1Buffer.getLength(); i++) {
      led1Buffer.setLED(i, Color.kRed);
    }
    led1.setData(led1Buffer);
  }
}
