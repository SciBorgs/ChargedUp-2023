package org.sciborgs1155.lib.constants;

public record ConversionConfig(double pulsesPerRev, double gearing, double units) {

  /**
   * An enum to represent common pulses per rotation presets
   *
   * <p>Note that there is a difference between PPR (pulses per revolution) and CPR (cycles per
   * revolution). In a quadrature encoder, the CPR will be 4 * PPR.
   */
  public enum PulsesPerRev {
    REV_THROUGHBORE(2048),
    REV_INTEGRATED(1);

    public final double pulses;

    private PulsesPerRev(double pulses) {
      this.pulses = pulses;
    }
  }

  /** An enum to represent common unit conversions for rotations to the respective units */
  public enum Units {
    RADIANS(2.0 * Math.PI),
    DEGREES(360.0);

    public final double conversion;

    private Units(double conversion) {
      this.conversion = conversion;
    }
  }

  public static ConversionConfig base() {
    return new ConversionConfig(1, 1, 1);
  }

  /**
   * The position factor for the encoder.
   *
   * @return The product of gearing and units per rotation, divded by pulses per revolution
   */
  public double factor() {
    return gearing * units / pulsesPerRev;
  }

  public ConversionConfig withPulsesPerRev(PulsesPerRev pulsesPerRev) {
    return withPulsesPerRev(pulsesPerRev.pulses);
  }

  public ConversionConfig withPulsesPerRev(double pulsesPerRev) {
    return new ConversionConfig(pulsesPerRev, gearing, units);
  }

  public ConversionConfig multiplyGearing(double ratio) {
    return new ConversionConfig(pulsesPerRev, gearing * ratio, units);
  }

  public ConversionConfig divideGearing(double ratio) {
    return new ConversionConfig(pulsesPerRev, gearing / ratio, units);
  }

  public ConversionConfig multiplyRadius(double radius) {
    return multiplyGearing(radius);
  }

  public ConversionConfig inverted() {
    return multiplyGearing(-1.0); // TODO might introduce bugs
  }

  public ConversionConfig withUnits(Units units) {
    return withUnits(units.conversion);
  }

  public ConversionConfig withUnits(double units) {
    return new ConversionConfig(pulsesPerRev, gearing, units);
  }
}
