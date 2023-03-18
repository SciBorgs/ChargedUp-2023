package org.sciborgs1155.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.led;
import org.sciborgs1155.robot.Ports;
import org.sciborgs1155.robot.commands.Scoring.GamePiece;

public class LED extends SubsystemBase {

  private AddressableLED led1;
  // private AddressableLED led2;
  private AddressableLEDBuffer led1Buffer;
  // private AddressableLEDBuffer led2Buffer;

  private int rainbowFirstPixelHue;

  // note: why is the maidens one called pg (purple green) and the sciborgs one called sb (sciborgs
  // i assume)?
  public enum LEDColor {
    RAINBOW,
    PG,
    BLUE,
    YELLOW
  }

  public LED() {
    led1 = new AddressableLED(Ports.Led.led1);
    led1Buffer = new AddressableLEDBuffer(Constants.led.buffer1Length);
    led1.setLength(led1Buffer.getLength());
    led1.setData(led1Buffer);
    led1.start();
    // note: what are you trying to do with this led sim?
    new AddressableLEDSim(led1);

    // led2 = new AddressableLED(Ports.LED.led2);
    // led2Buffer = new AddressableLEDBuffer(Constants.led.led2Buffer);
    // led2.setLength(Constants.led.led2Length);
    // led2.start();
  }

  public void setColor(LEDColor desiredColor) {
    switch (desiredColor) {
      case RAINBOW -> rainbow();
      case PG -> pgLED();
      case YELLOW -> coneLED();
      case BLUE -> cubeLED();
    }
  }

  // note: is there a reason that you're using setRGB now instead of setLED?
  // from looking at the source code, it seems like if setRGB works, setLED should work too
  private void coneLED() {
    for (int i = 0; i < led1Buffer.getLength(); i++) {
      led1Buffer.setRGB(i, led.yellow.getRed(), led.yellow.getGreen(), led.yellow.getBlue());
      // led2Buffer.setLED(i, Color.kYellow);
    }
    led1.setData(led1Buffer);
    // led2.setData(led2Buffer);
  }

  // note: there's block of code for setting the leds to a color (the for loop) that you're
  // rewriting a few times
  // it might be a good idea to write a separate function that takes in a Color and sets all the
  // leds in the strip to that color
  private void cubeLED() {
    for (int i = 0; i < led1Buffer.getLength(); i++) {
      led1Buffer.setRGB(i, led.blue.getRed(), led.blue.getGreen(), led.blue.getBlue());
      // led2Buffer.setLED(i, Color.kBlue);
    }
    led1.setData(led1Buffer);
    // led2.setData(led2Buffer);
  }

  public Command gamePieceLED(GamePiece gamePiece) {
    return Commands.runOnce(gamePiece == GamePiece.CONE ? this::coneLED : this::cubeLED, this);
  }

  public void rainbow() {
    for (var i = 0; i < led1Buffer.getLength(); i++) {
      final var hue = (rainbowFirstPixelHue + (i * 180 / led1Buffer.getLength())) % 180;

      led1Buffer.setHSV(i, hue, 255, 128);
    }
    rainbowFirstPixelHue += 3;
    rainbowFirstPixelHue %= 180;
  }

  /** Black and Yellow */
  public void sbLED() {
    for (int i = 0; i < led1Buffer.getLength(); i++) {
      led1Buffer.setRGB(i, led.yellow.getRed(), led.yellow.getGreen(), led.yellow.getBlue());
    }
    led1.setData(led1Buffer);

    // for (int i = 0; i < led2Buffer.getLength(); i++) {
    //   led2Buffer.setLED(i, Color.kYellow);
    // }
    // led2.setData(led2Buffer);
  }

  /** Purple and green */
  public void pgLED() {
    for (int i = 0; i < led1Buffer.getLength(); i++) {
      led1Buffer.setRGB(
          i, led.lightPurple.getRed(), led.lightPurple.getGreen(), led.lightPurple.getBlue());
    }
    led1.setData(led1Buffer);

    // for (int i = 0; i < led2Buffer.getLength(); i++) {
    //   led2Buffer.setLED(i, Color.kLimeGreen);
    // }
    // led2.setData(led2Buffer);
  }
}
