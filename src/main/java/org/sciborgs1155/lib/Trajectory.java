package org.sciborgs1155.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import java.util.List;

import org.sciborgs1155.robot.util.placement.PlacementState;

/** A generalized trajectory based off [position velocity] states. */
public class Trajectory {
  private final PlacementState initialPos, finalPos;
  private final List<Double> states;
  private final double totalTime;

  /**
   * Constructs a trajectory from a vector of states.
   *
   * @param states A vector of states.
   */
  public Trajectory(final PlacementState initialPos, final PlacementState finalPos, final List<Double> states, final double totalTime) {
    this.initialPos = initialPos;
    this.finalPos = finalPos;
    this.states = states;
    this.totalTime = totalTime;
  }

  public double getState(int index) {
    return states.get(index);
  }

  public double getLast() {
    return states.get(states.size() - 1);
  }

  public double getTotalTime() {
    return totalTime;
  }

  /** Samples from the trajectory, returning a */
  public Vector<N3> sample(double time) {
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
    double position = MathUtil.interpolate(states.get(prev), states.get(next), (time % dt) / dt);
    double velocity = (states.get(next) - states.get(prev)) / dt;

    double acceleration;
    if ((time % dt) / dt < 0.5) {
      var prevVelocity = (states.get(prev) - states.get(secondPrev)) / dt;
      acceleration = (velocity - prevVelocity) / dt;
    } else {
      var nextVelocity = (states.get(secondNext) - states.get(next)) / dt;
      acceleration = (nextVelocity - velocity) / dt;
    }

    // return state vector
    return VecBuilder.fill(position, velocity, acceleration);
  }

  /** Checks if the trajectory is within the tolerance of another one */
  // public boolean compare(Trajectory other, double tolerance) {
  //   return getState(0).isIdentical(other.getState(0), tolerance);
  // }
}
