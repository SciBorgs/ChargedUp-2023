package org.sciborgs1155.lib;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

public class WheelSim extends LinearSystemSim<N2, N1, N2> {
  /**
   * Create a state-space model for a 1(?) DOF "wheel" system from its kV (volts/(unit/sec)) and kA
   * (volts/(unit/sec²). These constants cam be found using SysId. The states of the system are
   * [position, velocity]ᵀ, inputs are [voltage], and outputs are [position, velocity].
   *
   * <p>The distance unit you choose MUST be an SI unit (i.e. meters or radians). You can use the
   * {@link edu.wpi.first.math.util.Units} class for converting between unit types.
   *
   * <p>The parameters provided by the user are from this feedforward model:
   *
   * <p>u = K_v v + K_a a
   *
   * <p>this is similar to {@link
   * edu.wpi.first.math.system.plant.LinearSystemId#identifyPositionSystem(double, double)}, but it
   * uses a (2x2) matrix to preserve position and velocity
   *
   * @param kV The velocity gain, in volts/(unit/sec)
   * @param kA The acceleration gain, in volts/(unit/sec²)
   * @see <a href="https://github.com/wpilibsuite/sysid">https://github.com/wpilibsuite/sysid</a>
   */
  public WheelSim(double kV, double kA) {
    super(identifySystem(kV, kA));
  }

  /** Output values for LinearSystemSim#getOutput */
  private enum Output {
    POSITION,
    VELOCITY
  }

  /**
   * Returns the wheel position.
   *
   * @return The wheel position in meters.
   */
  public double getPosition() {
    return super.getOutput(Output.POSITION);
  }
  
  /**
   * Returns the wheel velocity.
   *
   * @return The wheel velocity in rad / sec.
   */
  public double getVelocity() {
    return super.getOutput(Output.VELOCITY);
  }

  public void reset() {
    super.setState(Matrix.mat(Nat.N2(), Nat.N1()).fill(0, 0));
  }

  /**
   * Create a state-space model for a 1(?) DOF "wheel" system from its kV (volts/(unit/sec)) and kA
   * (volts/(unit/sec²). These constants cam be found using SysId. The states of the system are
   * [position, velocity]ᵀ, inputs are [voltage], and outputs are [position, velocity].
   *
   * <p>The distance unit you choose MUST be an SI unit (i.e. meters or radians). You can use the
   * {@link edu.wpi.first.math.util.Units} class for converting between unit types.
   *
   * <p>The parameters provided by the user are from this feedforward model:
   *
   * <p>u = K_v v + K_a a
   *
   * <p>this is similar to {@link
   * edu.wpi.first.math.system.plant.LinearSystemId#identifyPositionSystem(double, double)}, but it
   * uses a (2x2) matrix to preserve position and velocity
   *
   * @param kV The velocity gain, in volts/(unit/sec)
   * @param kA The acceleration gain, in volts/(unit/sec²)
   * @return A LinearSystem representing the given characterized constants.
   * @throws IllegalArgumentException if kV &lt;= 0 or kA &lt;= 0.
   * @see <a href="https://github.com/wpilibsuite/sysid">https://github.com/wpilibsuite/sysid</a>
   */
  public static LinearSystem<N2, N1, N2> identifySystem(double kV, double kA) {
    if (kV <= 0.0) {
      throw new IllegalArgumentException("Kv must be greater than zero.");
    }
    if (kA <= 0.0) {
      throw new IllegalArgumentException("Ka must be greater than zero.");
    }

    return new LinearSystem<N2, N1, N2>(
        Matrix.mat(Nat.N2(), Nat.N2()).fill(0.0, 1.0, 0.0, -kV / kA),
        Matrix.mat(Nat.N2(), Nat.N1()).fill(0, 1.0 / kA),
        Matrix.eye(Nat.N2()),
        Matrix.mat(Nat.N2(), Nat.N1()).fill(0.0, 0.0));
  }
}
