package org.sciborgs1155.robot.subsystems.LEDStrips;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public interface LEDStrip {

  public void setLength(int length);

  public void setData(AddressableLEDBuffer buffer);

  public void start();
}
