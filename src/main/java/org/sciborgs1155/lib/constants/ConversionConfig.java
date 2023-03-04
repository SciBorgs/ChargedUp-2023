package org.sciborgs1155.lib.constants;

public record ConversionConfig(double ppr, double gearing, double units) {

  /**
   * An enum to represent common pulses per rotation presets
   *
   * <p>Note that there is a difference between PPR (pulses per revolution) and CPR (cycles per
   * revolution). In a quadrature encoder, the CPR will be 4 * PPR.
   */
  public enum PPR {
    REV_THROUGHBORE(2048),
    REV_INTEGRATED(1);

    public final double pulses;

    private PPR(double pulses) {
      this.pulses = pulses;
    }
  }

  /** An enum to represent common unit conversions */
  public enum Units {
    RADIANS(2.0 * Math.PI),
    DEGREES(360.0);

    public final double conversion;

    private Units(double conversion) {
      this.conversion = conversion;
    }
  }

  /**
   * The position factor for the encoder.
   *
   * @return The product of gearing, units per rotation, and pulses per revolution
   */
  public double factor() {
    return gearing * units * ppr;
  }

  public ConversionConfig withPPR(PPR ppr) {
    return withPPR(ppr.pulses);
  }

  public ConversionConfig withPPR(double ppr) {
    return new ConversionConfig(ppr, gearing, units);
  }

  public ConversionConfig withGearing(double gearing) {
    return new ConversionConfig(ppr, gearing, units);
  }

  public ConversionConfig withUnits(Units units) {
    return withUnits(units.conversion);
  }

  public ConversionConfig withUnits(double units) {
    return new ConversionConfig(ppr, gearing, units);
  }

  public ConversionConfig multiplyGearing(double gearing) {
    return new ConversionConfig(ppr, this.gearing * gearing, units);
  }
}
