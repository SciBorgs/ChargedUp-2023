package org.sciborgs1155.lib.constants;

import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;

/** Record to store kP, kI, and kD values for PID controllers */
public record PIDConstants(double P, double I, double D) {

  /**
   * Creates a PID controller from these constants.
   *
   * @return A new WPILib {@link PIDController}.
   */
  public PIDController create() {
    return new PIDController(P, I, D);
  }

  /**
   * Sets the gains of a PID controller.
   *
   * @param controller A WPILib {@link PIDController}
   */
  public void set(PIDController controller) {
    controller.setP(P);
    controller.setI(I);
    controller.setD(D);
  }

  /**
   * Sets the gains of a PID controller.
   *
   * @param controller A REVLib {@link SparkMaxPIDController}
   */
  public void set(SparkMaxPIDController controller) {
    controller.setP(P);
    controller.setI(I);
    controller.setD(D);
  }
}
