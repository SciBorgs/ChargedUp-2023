// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.lib.constants;

import com.revrobotics.CANSparkMax;

/** Add your docs here. */
public class SparkConfig {
  private NeutralBehavior neutralBehavior;
  private boolean inverted = false;

  private SensorType sensor = SensorType.MISSING;
  private PIDConstants pid = new PIDConstants(0, 0, 0);
  private CANSparkMax lead = null;

  private int currentLimit = 80;
  private int openLoopRampRate = 0;

  public enum NeutralBehavior {
    COAST(true),
    BRAKE(false);

    private final boolean coast;

    private NeutralBehavior(boolean coast) {
      this.coast = coast;
    }

    public CANSparkMax.IdleMode getREV() {
      return coast ? CANSparkMax.IdleMode.kCoast : CANSparkMax.IdleMode.kBrake;
    }
  }

  public enum SensorType {
    MISSING,
    INTEGRATED;
  }

  public void setInverted(boolean inverted) {
    this.inverted = inverted;
  }

  public void setNeutralBehavior(NeutralBehavior neutralBehavior) {
    this.neutralBehavior = neutralBehavior;
  }

  public void setSensor(SensorType sensor) {
    this.sensor = sensor;
  }

  public void setPID(PIDConstants pid) {
    this.pid = pid;
  }

  public void setCurrentLimit(int currentLimit) {
    this.currentLimit = currentLimit;
  }

  public void setOpenLoopRampRate(int openLoopRampRate) {
    this.openLoopRampRate = openLoopRampRate;
  }

  public void follow(CANSparkMax lead) {
    this.lead = lead;
  }

  public boolean isInverted() {
    return inverted;
  }

  public NeutralBehavior getNeutralBehavior() {
    return neutralBehavior;
  }

  public SensorType getSensor() {
    return sensor;
  }

  public PIDConstants getPID() {
    return pid;
  }

  public int getCurrentLimit() {
    return currentLimit;
  }

  public int getOpenLoopRampRate() {
    return openLoopRampRate;
  }

  public CANSparkMax getLead() {
    return lead;
  }
}
