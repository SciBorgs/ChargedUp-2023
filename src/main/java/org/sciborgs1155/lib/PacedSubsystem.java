package org.sciborgs1155.lib;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class PacedSubsystem extends SubsystemBase {

  public PacedSubsystem(double rate) {
    FunctionRegistry.getInstance().add(this::pacedPeriodic, rate);
  }

  public PacedSubsystem() {
    this(FunctionRegistry.DEFAULT_RATE);
  }

  public abstract void pacedPeriodic();
}
