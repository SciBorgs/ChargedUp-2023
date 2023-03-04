package org.sciborgs1155.lib.constants;

import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;

/** Record to store kP, kI, and kD values for PID controllers */
public record PIDConstants(double kp, double ki, double kd) {

  /**
   * Creates a PID controller from these constants.
   *
   * @return A new WPILib {@link PIDController}.
   */
  public PIDController create() {
    return new PIDController(kp, ki, kd);
  }

  /**
   * Sets the gains of a PID controller.
   *
   * @param controller A WPILib {@link PIDController}
   */
  public void set(PIDController controller) {
    controller.setPID(kp, ki, kd);
  }

  /**
   * Sets the gains of a PID controller.
   *
   * @param controller A REVLib {@link SparkMaxPIDController}
   */
  public void set(SparkMaxPIDController controller) {
    controller.setP(kp);
    controller.setI(ki);
    controller.setD(kd);
  }
}
