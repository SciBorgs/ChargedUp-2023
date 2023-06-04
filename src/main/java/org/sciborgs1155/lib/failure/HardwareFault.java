package org.sciborgs1155.lib.failure;

import edu.wpi.first.wpilibj.Timer;

public record HardwareFault(String description, double timestamp, boolean isWarning) {

  public HardwareFault(String description, boolean isWarning) {
    this(description, Timer.getFPGATimestamp(), isWarning);
  }

  public HardwareFault(String description) {
    this(description, Timer.getFPGATimestamp(), false);
  }
}
