package org.sciborgs1155.robot.subsystems;

import static org.sciborgs1155.robot.Constants.Arm.*;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;
import org.sciborgs1155.lib.DeferredCommand;
import org.sciborgs1155.lib.Trajectory;
import org.sciborgs1155.robot.Constants.Elevator;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.subsystems.arm.ArmState;
import org.sciborgs1155.robot.subsystems.arm.ElevatorIO;
import org.sciborgs1155.robot.subsystems.arm.JointIO;
import org.sciborgs1155.robot.subsystems.arm.RealElbow;
import org.sciborgs1155.robot.subsystems.arm.RealElevator;
import org.sciborgs1155.robot.subsystems.arm.RealWrist;
import org.sciborgs1155.robot.subsystems.arm.SimElevator;
import org.sciborgs1155.robot.subsystems.arm.SimJoint;
import org.sciborgs1155.robot.subsystems.arm.TrajectoryCache;
import org.sciborgs1155.robot.subsystems.arm.TrajectoryCache.ArmTrajectory;
import org.sciborgs1155.robot.subsystems.arm.TrajectoryCache.Parameters;
import org.sciborgs1155.robot.subsystems.arm.Visualizer;

public class Arm extends SubsystemBase implements Loggable, AutoCloseable {

  public static record JointConfig(
      DCMotor gearbox,
      double gearing,
      double length,
      double mass,
      double minAngle,
      double maxAngle) {}

  public static record ElevatorConfig(
      DCMotor gearbox,
      double gearing,
      double mass,
      double sprocketRadius,
      double minHeight,
      double maxHeight) {}

  @Log private final ElevatorIO elevator;
  @Log private final JointIO elbow;
  @Log private final JointIO wrist;

  private final Map<Integer, ArmTrajectory> trajectories = TrajectoryCache.loadTrajectories();

  @Log private final Visualizer positionVisualizer = new Visualizer(new Color8Bit(255, 0, 0));
  @Log private final Visualizer setpointVisualizer = new Visualizer(new Color8Bit(0, 0, 255));

  @Log private boolean stopped = false;
  // 'tis but a scratch (see: match 42) (the elbow is unplugged)
  @Log private boolean butAScratch = false;
  @Log private boolean wristLimp = false;

  public Arm() {
    if (Robot.isReal()) {
      elevator = new RealElevator();
      elbow = new RealElbow(Elbow.PID, Elbow.FF);
      wrist = new RealWrist(Wrist.PID, Wrist.FF);
    } else {
      elevator = new SimElevator(Elevator.PID, Elevator.FF, Elevator.CONFIG, true);
      elbow = new SimJoint(Elbow.PID, Elbow.FF, Elbow.CONFIG, true);
      wrist = new SimJoint(Wrist.PID, Wrist.FF, Wrist.CONFIG, false);
    }
  }

  /** Returns the current position of the placement mechanisms */
  public ArmState getState() {
    return new ArmState(elevator.getHeight(), elbow.getRelativeAngle(), wrist.getRelativeAngle());
  }

  /** Returns the current setpoint of the placement mechanisms */
  public ArmState getSetpoint() {
    return ArmState.fromRelative(
        elevator.getDesiredState().position,
        elbow.getDesiredState().position,
        wrist.getDesiredState().position);
  }

  /**
   * If elbow is far enough from setpoint (we always use trapezoid profiling or trajectories), it is
   * dangerous. This method returns a debounced trigger for when the elbow encoder is likely
   * failing/not plugged in.
   */
  public Trigger onElbowFailing() {
    return new Trigger(() -> butAScratch).debounce(Elbow.FAILING_DEBOUNCE_TIME, DebounceType.kBoth);
  }

  /**
   * If wrist is not working, we cannot pass over. The wrist not working is determined in {@link
   * #periodic()}
   */
  @Log
  public boolean allowPassOver() {
    return !wristLimp;
  }

  /**
   * Finds a trajectory with the same start and end positions by hashing parameters.
   *
   * @param params Parameters to be hashed, contains start and end positions.
   * @return Optional placement trajectory, empty if the trajectory cannot be found from cashed
   *     values.
   */
  public Optional<ArmTrajectory> findTrajectory(Parameters params) {
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
  public Optional<ArmTrajectory> findTrajectory(ArmState goal) {
    return findTrajectory(new Parameters(getSetpoint(), goal));
  }

  public CommandBase goTo(ArmState goal) {
    return goTo(() -> goal);
  }

  /**
   * Goes to a {@link ArmState} in the most optimal way, this is a safe command.
   *
   * <p>Uses {@link #followTrajectory(ArmTrajectory)} based on {@link #findTrajectory(ArmState)} if
   * a valid state is cached for the inputted parameters. Otherwise, falls back on {@link
   * #safeFollowProfile(ArmState)} for on the fly movements.
   *
   * @param goal The goal state.
   * @param useTrajectories Whether to use trajectories.
   * @return A command that goes to the goal safely using either custom trajectory following or
   *     trapezoid profiling.
   */
  public CommandBase goTo(Supplier<ArmState> goal) {
    return new DeferredCommand(
            () ->
                Commands.either(
                    findTrajectory(goal.get())
                        .map(this::followTrajectory)
                        .orElse(safeFollowProfile(goal)),
                    Commands.none(),
                    () -> allowPassOver() || goal.get().side() == getState().side()),
            this)
        .withName("placement goto");
  }

  /**
   * A (mostly) safe version of {@link #followProfile(ArmState)} that uses {@link #passOver(Side)}
   * to reach the other side without height violations or destruction.
   *
   * <p>This is implemented by going to a safe intermediate goal if the side of the arm will change,
   * which is slow, and does not prevent circumstances where the arm hits the ground.
   *
   * @param goal The goal goal.
   * @return A safe following command that will run to a safe goal and then until all mechanisms are
   *     at their goal.
   */
  private CommandBase safeFollowProfile(Supplier<ArmState> goal) {
    return Commands.either(
            followProfile(() -> ArmState.passOverToSide(goal.get().side())),
            Commands.none(),
            () -> goal.get().side() != getState().side())
        .andThen(followProfile(goal))
        .withName("safe follow profile");
  }

  /** Sets the position setpoints for the elbow and wrist, in radians */
  public CommandBase setSetpoints(Supplier<ArmState> setpoint) {
    return runOnce(
            () -> {
              elevator.update(new State(setpoint.get().elevatorHeight(), 0));
              elbow.update(new State(setpoint.get().elbowAngle().getRadians(), 0));
              wrist.update(new State(setpoint.get().wristAngle().getRadians(), 0));
            })
        .withName("set setpoints");
  }

  /** Follows a {@link TrapezoidProfile} for each joint's relative position */
  private CommandBase followProfile(Supplier<ArmState> goal) {
    return new DeferredCommand(
            () ->
                Commands.parallel(
                    new TrapezoidProfileCommand(
                        new TrapezoidProfile(
                            Elevator.CONSTRAINTS,
                            new State(goal.get().elevatorHeight(), 0),
                            elevator.getState()),
                        elevator::update),
                    new TrapezoidProfileCommand(
                        new TrapezoidProfile(
                            Elbow.CONSTRAINTS,
                            new State(goal.get().elbowAngle().getRadians(), 0),
                            elbow.getCurrentState()),
                        elbow::update),
                    new TrapezoidProfileCommand(
                        new TrapezoidProfile(
                            Wrist.CONSTRAINTS,
                            new State(goal.get().wristAngle().getRadians(), 0),
                            wrist.getCurrentState()),
                        wrist::update)),
            this)
        .withName("following profile");
  }

  /** Follows a {@link Trajectory} for each joint's relative position */
  private CommandBase followTrajectory(ArmTrajectory trajectory) {
    return Commands.parallel(
            trajectory.elevator().follow(elevator::update),
            trajectory.elbow().follow(elbow::update),
            trajectory.wrist().follow(wrist::update, this))
        .withName("following trajectory");
  }

  public Command setStopped(boolean stopped) {
    return runOnce(() -> this.stopped = stopped);
  }

  @Override
  public void periodic() {
    positionVisualizer.setState(getState());
    setpointVisualizer.setState(getSetpoint());
  }

  @Override
  public void close() throws Exception {
    elevator.close();
    elbow.close();
    wrist.close();
  }
}
