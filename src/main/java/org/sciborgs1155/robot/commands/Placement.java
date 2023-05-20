package org.sciborgs1155.robot.commands;

import static org.sciborgs1155.robot.Constants.Positions.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;
import org.sciborgs1155.lib.DeferredCommand;
import org.sciborgs1155.robot.subsystems.Arm;
import org.sciborgs1155.robot.util.placement.PlacementCache;
import org.sciborgs1155.robot.util.placement.PlacementState;
import org.sciborgs1155.robot.util.placement.PlacementState.Side;
import org.sciborgs1155.robot.util.placement.PlacementTrajectory;
import org.sciborgs1155.robot.util.placement.PlacementTrajectory.Parameters;

/** Placement command factories */
public final class Placement {

  private final Arm arm;
  private final ElevatorIO elevator;

  private final Map<Integer, PlacementTrajectory> trajectories = PlacementCache.loadTrajectories();

  public Placement(Arm arm, ElevatorIO elevator) {
    this.arm = arm;
    this.elevator = elevator;
  }

  /**
   * Finds a trajectory with the same start and end positions by hashing parameters.
   *
   * @param params Parameters to be hashed, contains start and end positions.
   * @return Optional placement trajectory, empty if the trajectory cannot be found from cashed
   *     values.
   */
  public Optional<PlacementTrajectory> findTrajectory(Parameters params) {
    return Optional.ofNullable(trajectories.get(params.hashCode()));
  }

  /**
   * Finds a trajectory with the same current setpoint as the start and goal as end positions by
   * hashing parameters.
   *
   * @param goal The desired state of the placement mechanisms.
   * @return Optional placement trajectory, empty if the trajectory cannot be found from cashed
   *     values.
   */
  public Optional<PlacementTrajectory> findTrajectory(PlacementState goal) {
    return findTrajectory(new Parameters(setpoint(), goal));
  }

  /**
   * Returns a goal based on the side being moved to. This is used for on the fly trapezoidal
   * control.
   *
   * @param side The side of the robot being moved to.
   * @return The "passing over" goal according to the inputted side.
   */
  public PlacementState passOver(Side side) {
    return side == Side.FRONT ? PASS_TO_FRONT : PASS_TO_BACK;
  }

  public Command goTo(PlacementState goal) {
    return goTo(() -> goal);
  }

  /**
   * Goes to a {@link PlacementState} in the most optimal way, this is a safe command.
   *
   * <p>Uses {@link #followTrajectory(PlacementTrajectory)} based on {@link
   * #findTrajectory(PlacementState)} if a valid state is cached for the inputted parameters.
   * Otherwise, falls back on {@link #safeFollowProfile(PlacementState)} for on the fly movements.
   *
   * @param goal The goal state.
   * @param useTrajectories Whether to use trajectories.
   * @return A command that goes to the goal safely using either custom trajectory following or
   *     trapezoid profiling.
   */
  public Command goTo(Supplier<PlacementState> goal) {
    return new DeferredCommand(
            () ->
                Commands.either(
                    findTrajectory(goal.get())
                        .map(this::followTrajectory)
                        .orElse(safeFollowProfile(goal.get())),
                    Commands.none(),
                    () -> arm.allowPassOver() || goal.get().side() == state().side()),
            arm,
            elevator)
        .withName("placement goto");
  }

  /**
   * A (mostly) safe version of {@link #followProfile(PlacementState)} that uses {@link
   * #passOver(Side)} to reach the other side without height violations or destruction.
   *
   * <p>This is implemented by going to a safe intermediate goal if the side of the arm will change,
   * which is slow, and does not prevent circumstances where the arm hits the ground.
   *
   * @param goal The goal goal.
   * @return A safe following command that will run to a safe goal and then until all mechanisms are
   *     at their goal.
   */
  private Command safeFollowProfile(PlacementState goal) {
    return Commands.either(
            followProfile(passOver(goal.side())),
            Commands.none(),
            () -> goal.side() != state().side())
        .andThen(followProfile(goal))
        .withName("safe follow profile");
  }
  
  /** Sets everything to stopped */
  public Command setStopped(boolean stopped) {
    return Commands.parallel(elevator.setStopped(stopped), arm.setStopped(stopped));
  }
}
