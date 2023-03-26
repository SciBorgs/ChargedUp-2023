package org.sciborgs1155.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import java.util.List;

/** A generalized trajectory based off [position velocity] states. */
public class Trajectory {
  private final List<Vector<N2>> states;
  private final double totalTime;

  /**
   * Constructs a trajectory from a vector of states.
   *
   * @param states A vector of states.
   */
  public Trajectory(final List<Vector<N2>> states) {
    this.states = states;
    totalTime = 0.0;
  }

  /** Performs linear interpolation between two N dimensional vectors */
  private static final <N extends Num> Vector<N> lerp(Vector<N> start, Vector<N> end, double t) {
    var res = new Vector<>(start);
    for (int row = 0; row < res.getNumRows(); row++) {
      res.set(row, 0, MathUtil.interpolate(start.get(row, 0), end.get(row, 0), t));
    }
    return res;
  }

  /** Finds the slope between two N dimensional vectors */
  private static final <N extends Num> Vector<N> slope(Vector<N> start, Vector<N> end, double dt) {
    var res = new Vector<>(start);
    for (int row = 0; row < res.getNumRows(); row++) {
      res.set(row, 0, (end.get(row, 0) - start.get(row, 0)) / dt);
    }
    return res;
  }

  public Vector<N2> getState(int index) {
    return states.get(index);
  }

  /** Samples from the trajectory, returning a */
  public Matrix<N2, N3> sample(double time) {
    double dt = totalTime / (states.size() - 1);

    // surrounding indices
    int prev = (int) Math.floor(time / dt);
    int next = (int) Math.ceil(time / dt);
    if (next == prev) next++;
    int secondPrev = prev - 1;
    int secondNext = next + 1;

    // clamp indices
    prev = MathUtil.clamp(prev, 0, states.size() - 1);
    next = MathUtil.clamp(next, 0, states.size() - 1);
    secondPrev = MathUtil.clamp(secondPrev, 0, states.size() - 1);
    secondNext = MathUtil.clamp(secondNext, 0, states.size() - 1);

    // calculate values
    var position = lerp(states.get(prev), states.get(next), (time % dt) / dt);
    var velocity = slope(states.get(prev), states.get(next), dt);

    Vector<N2> acceleration;
    if ((time % dt) / dt < 0.5) {
      var prevVelocity = slope(states.get(secondPrev), states.get(prev), dt);
      acceleration = slope(prevVelocity, velocity, dt);
    } else {
      var nextVelocity = slope(states.get(next), states.get(secondNext), dt);
      acceleration = slope(velocity, nextVelocity, dt);
    }

    // return sample matrix
    var res = new Matrix<>(Nat.N2(), Nat.N3());
    res.assignBlock(0, 0, position);
    res.assignBlock(1, 0, velocity);
    res.assignBlock(2, 0, acceleration);
    return res;
  }

  /** Checks if the trajectory is within the tolerance of another one */
  public boolean compare(Trajectory other, double tolerance) {
    return getState(0).isIdentical(other.getState(0), tolerance);
  }
}
