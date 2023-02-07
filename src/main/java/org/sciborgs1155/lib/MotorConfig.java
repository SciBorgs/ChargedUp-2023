package org.sciborgs1155.lib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * MotorConfig is a builder class for standardizing vendor motor controllers.
 *
 * <p>Example usage for a CANSparkMax differential drive:
 *
 * <pre><code>
 * var leftConfig = MotorConfig
 *  .base()
 *  .withNeutralBehavior(NeutralBehavior.BRAKE)
 *  .withCurrentLimit(20);
 *
 * var rightConfig = leftMotor.withInverted(true);
 *
 * int[] leftPorts = { 1, 2, 3 };
 * int[] rightPorts = { 4, 5, 6 };
 *
 * var leftMotor = leftConfig.buildCanSparkMaxGearbox(MotorType.kBrushless, leftPorts);
 * var rightGroup = rightConfig.buildCanSparkMaxGearbox(MotorType.kBrushless, rightPorts);
 *
 * var drive = new DifferentialDrive(leftMotor, rightGroup);
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

  public final boolean inverted;
  public final NeutralBehavior neutralBehavior;
  public final double openLoopRampRate;
  public final int currentLimit;
  public final boolean burnFlash;

  public MotorConfig(
      boolean inverted,
      NeutralBehavior neutralBehavior,
      double openLoopRampRate,
      int currentLimit,
      boolean burnFlash) {
    this.inverted = inverted;
    this.neutralBehavior = neutralBehavior;
    this.openLoopRampRate = openLoopRampRate;
    this.currentLimit = currentLimit;
    this.burnFlash = burnFlash;
  }

  public static MotorConfig base() {
    return new MotorConfig(false, NeutralBehavior.COAST, 0, 80, true);
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
    motor.setIdleMode(neutralBehavior.getREV());
    motor.setOpenLoopRampRate(openLoopRampRate);
    motor.setSmartCurrentLimit(currentLimit);
    if (burnFlash) motor.burnFlash();
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
    if (ids.length < 1) {
      throw new IllegalArgumentException("Number of inputted ids is less than 1.");
    }

    var res = new CANSparkMax[ids.length];

    for (int i = 0; i < ids.length; i++) {
      res[i] = buildCanSparkMax(motorType, ids[i]);
    }

    return res;
  }

  /**
   * Returns a lead motor with others bound to follow each other based on their configured values
   *
   * <p>One motor controller will be created per id, in order
   *
   * @param motorType the rev motor type
   * @param ids a variable number of ids
   * @return The lead motor as a CANSparkMax object
   */
  public CANSparkMax buildCanSparkMaxGearbox(MotorType motorType, int... ids) {
    if (ids.length < 1) {
      throw new IllegalArgumentException("Number of inputted ids is less than 1.");
    }

    var lead = buildCanSparkMax(motorType, ids[0]);

    for (int i = 1; i < ids.length; i++) {
      buildCanSparkMax(motorType, ids[i]).follow(lead);
    }

    return lead;
  }

  public MotorConfig withInvert(boolean inverted) {
    return new MotorConfig(inverted, neutralBehavior, openLoopRampRate, currentLimit, burnFlash);
  }

  public MotorConfig withNeutralBehavior(NeutralBehavior neutralBehavior) {
    return new MotorConfig(inverted, neutralBehavior, openLoopRampRate, currentLimit, burnFlash);
  }
  public MotorConfig withOpenLoopRampRate(double openLoopRampRate) {
    return new MotorConfig(inverted, neutralBehavior, openLoopRampRate, currentLimit, burnFlash);
  }

  public MotorConfig withCurrentLimit(int currentLimit) {
    return new MotorConfig(inverted, neutralBehavior, openLoopRampRate, currentLimit, burnFlash);
  }
  
  public MotorConfig withBurnFlash(boolean burnFlash) {
    return new MotorConfig(inverted, neutralBehavior, openLoopRampRate, currentLimit, burnFlash);
  }
}
