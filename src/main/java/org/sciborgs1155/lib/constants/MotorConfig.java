package org.sciborgs1155.lib.constants;

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
public record MotorConfig(
    boolean inverted, NeutralBehavior neutralBehavior, double openLoopRampRate, int currentLimit) {

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

  public static MotorConfig base() {
    return new MotorConfig(false, NeutralBehavior.COAST, 0, 80);
  }

  /**
   * Creates a CANSparkMax based on configured values
   *
   * @param motorType the rev motor type
   * @param id the motor controller's device id
   * @return a new CANSparkMax object
   */
  public CANSparkMax build(MotorType motorType, int id) {
    var motor = new CANSparkMax(id, motorType);
    motor.restoreFactoryDefaults();
    motor.setInverted(inverted);
    motor.setIdleMode(neutralBehavior.getREV());
    motor.setOpenLoopRampRate(openLoopRampRate);
    motor.setSmartCurrentLimit(currentLimit);
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
  public CANSparkMax[] build(MotorType motorType, int... ids) {
    if (ids.length < 1) {
      throw new IllegalArgumentException("Number of inputted ids is less than 1.");
    }

    var res = new CANSparkMax[ids.length];

    for (int i = 0; i < ids.length; i++) {
      res[i] = build(motorType, ids[i]);
    }

    return res;
  }

  public MotorConfig withInvert(boolean inverted) {
    return new MotorConfig(inverted, neutralBehavior, openLoopRampRate, currentLimit);
  }

  public MotorConfig withNeutralBehavior(NeutralBehavior neutralBehavior) {
    return new MotorConfig(inverted, neutralBehavior, openLoopRampRate, currentLimit);
  }

  public MotorConfig withOpenLoopRampRate(double openLoopRampRate) {
    return new MotorConfig(inverted, neutralBehavior, openLoopRampRate, currentLimit);
  }

  public MotorConfig withCurrentLimit(int currentLimit) {
    return new MotorConfig(inverted, neutralBehavior, openLoopRampRate, currentLimit);
  }
}
