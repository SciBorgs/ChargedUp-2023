// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.lib.constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/** Add your docs here. */
public class SparkConfig {
  private final CANSparkMax motor;

  public SparkConfig(int port, MotorType motorType) {
    motor = new CANSparkMax(port, motorType);
    motor.restoreFactoryDefaults();
  }

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

  // TODO: Work out specific logic for sensors
  public void setInverted(boolean inverted) {
    motor.setInverted(inverted);
  }

  public void setNeutralBehavior(NeutralBehavior neutralBehavior) {
    motor.setIdleMode(neutralBehavior.getREV());
  }

  public void setSensor(SensorType sensor) {}

  public void setPID(PIDConstants pid) {}

  public void setCurrentLimit(int currentLimit) {
    motor.setSmartCurrentLimit(currentLimit);
  }

  public void setOpenLoopRampRate(double openLoopRampRate) {
    motor.setOpenLoopRampRate(openLoopRampRate);
  }

  public void follow(CANSparkMax lead) {
    motor.follow(lead);
  }

  /** Creates a CANSparkMax based on configured values */
  public CANSparkMax build() {
    return motor;
  }
}
