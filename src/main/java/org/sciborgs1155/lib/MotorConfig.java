package org.sciborgs1155.lib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * MotorConfig is a builder class for standardizing vendor motor controllers.
 *
 * <p>Example usage for a CANSparkMax differential drive:
 *
 * <pre><code>
 * var leftMotor = new MotorConfig()
 *  .setNeutralBehavior(NeutralBehavior.BRAKE)
 *  .setCurrentLimit(20);
 *
 * var rightMotor = leftMotor.clone()
 *  .setInverted(true);
 *
 * int[] leftPorts = {1, 2, 3};
 * int[] rightPorts = {4, 5, 6};
 *
 * var leftGroup = new MotorControllerGroup(leftMotor.buildCanSparkMax(leftPorts));
 * var rightGroup = new MotorControllerGroup(rightMotor.buildCanSparkMax(rightPorts));
 *
 * var driveTrain = new DifferentialDrive(leftGroup, rightGroup);
 * </pre></code>
 */
public final class MotorConfig {

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

  private boolean inverted;
  private NeutralBehavior neutralBehavior;
  private double openLoopRampRate;
  private int currentLimit;

  public MotorConfig() {
    setInverted(false);
    setNeutralBehavior(NeutralBehavior.COAST);
    setOpenLoopRampRate(0);
    setCurrentLimit(80);
  }

  /**
   * Creates a CANSparkMax based on configured values
   *
   * @param motorType the rev motor type
   * @param id the motor controller's device id
   * @return a new CANSparkMax object
   */
  public CANSparkMax buildCanSparkMax(MotorType motorType, int id) {
    var motor = new CANSparkMax(id, motorType);
    motor.restoreFactoryDefaults();
    motor.setInverted(inverted);
    motor.setOpenLoopRampRate(openLoopRampRate);
    motor.setSmartCurrentLimit(currentLimit);
    motor.burnFlash();
    return motor;
  }

  /**
   * Creates a list of CANSparkMax objects based on their configured values
   *
   * <p>One motor controller will be created per id, in order
   *
   * @param motorType the rev motor type
   * @param ids a variable number of ids
   * @return array of CANSparkMax objects
   */
  public CANSparkMax[] buildCanSparkMax(MotorType motorType, int... ids) {
    var res = new CANSparkMax[ids.length];
    for (int i = 0; i < ids.length; i++) {
      res[i] = buildCanSparkMax(motorType, ids[i]);
    }
    return res;
  }

  public boolean isInverted() {
    return inverted;
  }

  public MotorConfig setInverted(boolean inverted) {
    this.inverted = inverted;
    return this;
  }

  public NeutralBehavior getNeutralBehavior() {
    return neutralBehavior;
  }

  public MotorConfig setNeutralBehavior(NeutralBehavior neutralBehavior) {
    this.neutralBehavior = neutralBehavior;
    return this;
  }

  public double getOpenLoopRampRate() {
    return openLoopRampRate;
  }

  public MotorConfig setOpenLoopRampRate(double openLoopRampRate) {
    this.openLoopRampRate = openLoopRampRate;
    return this;
  }

  public int getCurrentLimit() {
    return currentLimit;
  }

  public MotorConfig setCurrentLimit(int currentLimit) {
    this.currentLimit = currentLimit;
    return this;
  }

  public MotorConfig clone() {
    return new MotorConfig()
        .setInverted(isInverted())
        .setNeutralBehavior(getNeutralBehavior())
        .setOpenLoopRampRate(getOpenLoopRampRate())
        .setCurrentLimit(getCurrentLimit());
  }
}
