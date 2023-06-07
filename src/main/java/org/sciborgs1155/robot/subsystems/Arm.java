package org.sciborgs1155.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;
import org.sciborgs1155.lib.DeferredCommand;
import org.sciborgs1155.lib.Trajectory;
import org.sciborgs1155.lib.failure.Fallible;
import org.sciborgs1155.lib.failure.FaultBuilder;
import org.sciborgs1155.lib.failure.HardwareFault;
import org.sciborgs1155.robot.Constants.Elbow;
import org.sciborgs1155.robot.Constants.Elevator;
import org.sciborgs1155.robot.Constants.Wrist;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.subsystems.arm.ArmState;
import org.sciborgs1155.robot.subsystems.arm.ArmState.GamePiece;
import org.sciborgs1155.robot.subsystems.arm.ArmState.Goal;
import org.sciborgs1155.robot.subsystems.arm.ElevatorIO;
import org.sciborgs1155.robot.subsystems.arm.ElevatorIO.ElevatorConfig;
import org.sciborgs1155.robot.subsystems.arm.JointIO;
import org.sciborgs1155.robot.subsystems.arm.JointIO.JointConfig;
import org.sciborgs1155.robot.subsystems.arm.NoElevator;
import org.sciborgs1155.robot.subsystems.arm.NoJoint;
import org.sciborgs1155.robot.subsystems.arm.RealElbow;
import org.sciborgs1155.robot.subsystems.arm.RealElevator;
import org.sciborgs1155.robot.subsystems.arm.RealWrist;
import org.sciborgs1155.robot.subsystems.arm.SimElevator;
import org.sciborgs1155.robot.subsystems.arm.SimJoint;
import org.sciborgs1155.robot.subsystems.arm.TrajectoryCache;
import org.sciborgs1155.robot.subsystems.arm.TrajectoryCache.ArmTrajectory;
import org.sciborgs1155.robot.subsystems.arm.TrajectoryCache.Parameters;
import org.sciborgs1155.robot.subsystems.arm.Visualizer;

public class Arm extends SubsystemBase implements Fallible, Loggable, AutoCloseable {

  public static Arm createNone() {
    return new Arm(new NoElevator(), new NoJoint(), new NoJoint());
  }

  public static Arm createFromConfig(
      ElevatorConfig elevator, JointConfig elbow, JointConfig wrist) {
    return Robot.isReal()
        ? new Arm(new RealElevator(elevator), new RealElbow(elbow), new RealWrist(wrist))
        : new Arm(new SimElevator(elevator), new SimJoint(elbow, true), new SimJoint(wrist, false));
  }

  @Log private final ElevatorIO elevator;
  @Log private final JointIO elbow;
  @Log private final JointIO wrist;

  private final Map<Integer, ArmTrajectory> trajectories = TrajectoryCache.loadTrajectories();

  @Log private final Visualizer positionVisualizer = new Visualizer(new Color8Bit(255, 0, 0));
  @Log private final Visualizer setpointVisualizer = new Visualizer(new Color8Bit(0, 0, 255));

  // this is to make the emperical collection of arm states easier
  @Log(name = "copy state")
  String copyState = "";

  public Arm(ElevatorIO elevator, JointIO elbow, JointIO wrist) {
    this.elevator = elevator;
    this.elbow = elbow;
    this.wrist = wrist;
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
   * Finds a trajectory with the same current setpoint as the start and goal as end positions by
   * hashing parameters.
   *
   * @param goal The desired state of the placement mechanisms.
   * @return Optional placement trajectory, empty if the trajectory cannot be found from cashed
   *     values.
   */
  public Optional<ArmTrajectory> findTrajectory(ArmState goal) {
    return Optional.ofNullable(trajectories.get(new Parameters(getSetpoint(), goal).hashCode()));
  }

  /**
   * Goes to a {@link ArmState} in the most optimal way, this is a safe command.
   *
   * <p>Uses {@link #followTrajectory(ArmTrajectory)} based on {@link #findTrajectory(ArmState)} if
   * a valid state is cached for the inputted parameters. Otherwise, falls back on {@link
   * #safeFollowProfile(ArmState)} for on the fly movements.
   *
   * @param goal The goal state.
   * @param gamePiece The selected game piece.
   * @return A command that goes to the goal safely using either custom trajectory following or
   *     trapezoid profiling.
   */
  public CommandBase goTo(Goal goal, Supplier<GamePiece> gamePiece) {
    return goTo(() -> ArmState.fromGoal(goal, gamePiece.get()));
  }

  /**
   * Goes to a {@link ArmState} in the most optimal way, this is a safe command.
   *
   * <p>Uses {@link #followTrajectory(ArmTrajectory)} based on {@link #findTrajectory(ArmState)} if
   * a valid state is cached for the inputted parameters. Otherwise, falls back on {@link
   * #safeFollowProfile(ArmState)} for on the fly movements.
   *
   * @param goal The goal state.
   * @return A command that goes to the goal safely using either custom trajectory following or
   *     trapezoid profiling.
   */
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
   * @param goal The goal state supplier.
   * @return A command that goes to the goal safely using either custom trajectory following or
   *     trapezoid profiling.
   */
  public CommandBase goTo(Supplier<ArmState> goal) {
    return new DeferredCommand(
        () ->
            findTrajectory(goal.get()).map(this::followTrajectory).orElse(safeFollowProfile(goal)),
        this);
  }

  /**
   * A (mostly) safe version of {@link #followProfile(ArmState)} that uses {@link ArmState#side()}
   * and {@link ArmState#end()} to reach the other side without height violations or destruction.
   *
   * <p>This is implemented by going to a safe intermediate goal if the side of the arm will change,
   * which is slow, and mostly prevents circumstances where the arm hits the ground.
   *
   * @param goal The goal goal.
   * @return A safe following command that will run to a safe goal and then until all mechanisms are
   *     at their goal.
   */
  private CommandBase safeFollowProfile(Supplier<ArmState> goal) {
    var toSide =
        Commands.either(
            followProfile(() -> ArmState.passOverToSide(goal.get().side())),
            Commands.none(),
            () -> goal.get().side() != getState().side());

    var toOrientation =
        Commands.either(
            followProfile(
                () ->
                    ArmState.fromRelative(
                        getState().elevatorHeight(),
                        goal.get().side().angle,
                        goal.get().wristAngle().getRadians())),
            Commands.none(),
            () -> getState().end() != goal.get().end());

    return toSide
        .andThen(toOrientation)
        .andThen(followProfile(goal))
        .withName("safe following profile");
  }

  /** Sets the position setpoints for the elbow and wrist, in radians */
  public CommandBase setSetpoints(Supplier<ArmState> setpoint) {
    return runOnce(
            () -> {
              elevator.updateSetpoint(new State(setpoint.get().elevatorHeight(), 0));
              elbow.updateSetpoint(new State(setpoint.get().elbowAngle().getRadians(), 0));
              wrist.updateSetpoint(new State(setpoint.get().wristAngle().getRadians(), 0));
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
                            elevator.getCurrentState()),
                        elevator::updateSetpoint),
                    new TrapezoidProfileCommand(
                        new TrapezoidProfile(
                            Elbow.CONSTRAINTS,
                            new State(goal.get().elbowAngle().getRadians(), 0),
                            elbow.getCurrentState()),
                        elbow::updateSetpoint),
                    new TrapezoidProfileCommand(
                        new TrapezoidProfile(
                            Wrist.CONSTRAINTS,
                            new State(goal.get().wristAngle().getRadians(), 0),
                            wrist.getCurrentState()),
                        wrist::updateSetpoint)),
            this)
        .withName("following profile");
  }

  /** Follows a {@link Trajectory} for each joint's relative position */
  private CommandBase followTrajectory(ArmTrajectory trajectory) {
    return Commands.parallel(
            trajectory.elevator().follow(elevator::updateSetpoint),
            trajectory.elbow().follow(elbow::updateSetpoint),
            trajectory.wrist().follow(wrist::updateSetpoint, this))
        .withName("following trajectory");
  }

  /** Disables the arm for the rest of the match */
  public CommandBase kill() {
    return run(() -> {
          elevator.stopMoving();
          elbow.stopMoving();
          wrist.stopMoving();
        })
        .alongWith(Commands.print("arm is fucked"))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .withName("killed");
  }

  @Override
  public List<HardwareFault> getFaults() {
    return FaultBuilder.create()
        .register(elevator.getFaults())
        .register(elbow.getFaults())
        .register(wrist.getFaults())
        .build();
  }

  @Override
  public void periodic() {
    var endpoint = getState().getEndpoint();
    copyState =
        String.format(
            "fromEndpoint(%.3f, %.3f, %.3f)",
            endpoint.getX(), endpoint.getY(), endpoint.getRotation().getRadians());

    wrist.setBaseAngle(elbow.getRelativeAngle());
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
