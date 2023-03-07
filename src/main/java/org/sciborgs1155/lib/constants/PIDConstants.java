package org.sciborgs1155.lib.constants;

import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;

/** Record to store P, I, and D gains for PID controllers */
public record PIDConstants(double p, double i, double d) {

  /**
   * Creates a PID controller from these constants.
   *
   * @return A new WPILib {@link PIDController}.
   */
  public PIDController create() {
    return new PIDController(p, i, d);
  }

  /**
   * Sets the gains of a PID controller.
   *
   * @param controller A WPILib {@link PIDController}
   */
  public void set(PIDController controller) {
    controller.setPID(p, i, d);
  }

  /**
   * Sets the gains of a PID controller.
   *
   * @param controller A REVLib {@link SparkMaxPIDController}
   */
  public void set(SparkMaxPIDController controller) {
    controller.setP(p);
    controller.setI(i);
    controller.setD(d);
  }
}
