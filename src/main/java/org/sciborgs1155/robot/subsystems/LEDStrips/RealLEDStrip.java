package org.sciborgs1155.robot.subsystems.LEDStrips;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class RealLEDStrip implements LEDStrip {
  AddressableLED strip;

  public RealLEDStrip(int port) {
    strip = new AddressableLED(port);
  }

  public void setLength(int length) {
    strip.setLength(length);
  }

  public void setData(AddressableLEDBuffer buffer) {
    strip.setData(buffer);
  }

  public void start() {
    strip.start();
  }
}
