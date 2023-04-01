package org.sciborgs1155.robot.commands;

import static org.sciborgs1155.robot.Constants.Positions.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import java.util.Map;
import java.util.Optional;
import org.sciborgs1155.robot.subsystems.Arm;
import org.sciborgs1155.robot.subsystems.Elevator;
import org.sciborgs1155.robot.util.placement.PlacementCache;
import org.sciborgs1155.robot.util.placement.PlacementState;
import org.sciborgs1155.robot.util.placement.PlacementState.Side;
import org.sciborgs1155.robot.util.placement.PlacementTrajectory;
import org.sciborgs1155.robot.util.placement.PlacementTrajectory.Parameters;

/** Placement command factories */
public final class Placement {

  private final Arm arm;
  private final Elevator elevator;

  private final Map<Integer, PlacementTrajectory> trajectories = PlacementCache.loadTrajectories();

  public Placement(Arm arm, Elevator elevator) {
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
    return findTrajectory(new Parameters(state(), goal));
  }

  /** Returns the current position goal of the placement mechanisms */
  public PlacementState state() {
    return PlacementState.fromRelative(
        elevator.getPosition(),
        arm.getElbowPosition().getRadians(),
        arm.getRelativeWristPosition().getRadians());
  }

  /** Returns the current position setpoint of placement mechanisms */
  public PlacementState setpoint() {
    return PlacementState.fromRelative(
        elevator.getSetpoint().position(),
        arm.getElbowSetpoint().position(),
        arm.getWristSetpoint().position());
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

  /**
   * Goes to a {@link PlacementState} in the most optimal way, this is a safe command.
   *
   * <p>Uses {@link #followTrajectory(PlacementTrajectory)} based on {@link
   * #findTrajectory(PlacementState)} if a valid state is cached for the inputted parameters.
   * Otherwise, falls back on {@link #safeFollowProfile(PlacementState)} for on the fly movements.
   *
   * @param goal The goal state.
   * @return A command that goes to the goal safely using either custom trajectory following or
   *     trapezoid profiling.
   */
  public Command goTo(PlacementState goal) {
    return new ProxyCommand(
        () ->
            findTrajectory(goal).isPresent()
                ? followTrajectory(findTrajectory(goal).get())
                : safeFollowProfile(goal));
  }

  /**
   * Runs elevator and arm to a {@link PlacementState}.
   *
   * <p><b>WARNING: THIS IS UNSAFE AND DOES NOT ACCOUNT FOR HITTING THE ROBOT</b>
   *
   * @param goal The goal goal.
   * @return A following command that will run until all mechanisms are at their goal.
   */
  public Command followProfile(PlacementState goal) {
    return Commands.parallel(
            elevator.followProfile(goal.elevatorHeight()),
            arm.followProfile(goal.elbowAngle(), goal.wristAngle()))
        .withName("placement follow profile");
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
  public Command safeFollowProfile(PlacementState goal) {
    return Commands.either(
            followProfile(passOver(goal.side())),
            Commands.none(),
            () -> goal.side() != state().side())
        .andThen(followProfile(goal));
  }

  /**
   * Follows generated trajctories based on a trajectory.
   *
   * <p>This is safe and efficient, and should always be prefered to {@link
   * #safeFollowProfile(PlacementState)}, unless there is no usable cached trajectory.
   *
   * @param trajectory The trajectory to follow.
   * @return A command to follow a generated trajectory.
   */
  public Command followTrajectory(PlacementTrajectory trajectory) {
    return Commands.parallel(
        elevator.followTrajectory(trajectory.elevator()),
        arm.followTrajectory(trajectory.elbow(), trajectory.wrist()));
  }
}
