package org.sciborgs1155.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.List;
import java.util.function.Consumer;

/** A generalized trajectory based off [position velocity] states. */
public class Trajectory {

  private final List<Double> states;
  private final double totalTime;

  /**
   * Constructs a trajectory from a vector of states.
   *
   * @param states A vector of states.
   */
  public Trajectory(final List<Double> states, final double totalTime) {
    this.states = states;
    this.totalTime = totalTime;
  }

  public double getState(int index) {
    return states.get(index);
  }

  public double getFirst() {
    return states.get(0);
  }

  public double getLast() {
    return states.get(states.size() - 1);
  }

  public double totalTime() {
    return totalTime;
  }

  /** Samples from the trajectory, returning a */
  public State sample(double time) {
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

    // return state vector
    return new State(position, velocity);
  }

  /** Returns a command to follow the trajectory using {@link TrajectoryCommand} */
  public CommandBase follow(Consumer<State> output, Subsystem... requirements) {
    return new TrajectoryCommand(this, output, requirements);
  }
}
