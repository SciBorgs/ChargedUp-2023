package org.sciborgs1155.lib;

public class BalanceFeedForward {
  public final double ks;
  public final double kv;
  public final double ka;
  public final double kg;

  public BalanceFeedForward(double ks, double kv, double ka, double kg) {
    this.ks = ks;
    this.kv = kv;
    this.ka = ka;
    this.kg = kg;
  }

  public BalanceFeedForward(double ks, double kv, double kg) {
    this(ks, kv, 0, kg);
  }

  public double calculate(double pitch, double velocity, double acceleration) {
    return ks * Math.signum(velocity) + kv * velocity + ka * acceleration + Math.sin(pitch) * kg;
  }

  public double calculate(double pitch, double velocity) {
    return calculate(pitch, velocity, 0);
  }
}
