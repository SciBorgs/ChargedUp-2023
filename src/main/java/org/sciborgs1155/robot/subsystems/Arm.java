package org.sciborgs1155.robot.subsystems;

import static org.sciborgs1155.robot.Constants.Arm.*;
import static org.sciborgs1155.robot.Ports.Arm.*;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.sciborgs1155.lib.DeferredCommand;
import org.sciborgs1155.lib.Trajectory;
import org.sciborgs1155.robot.Constants.Elevator;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.subsystems.arm.ElbowSparkMax;
import org.sciborgs1155.robot.subsystems.arm.ElevatorIOSim;
import org.sciborgs1155.robot.subsystems.arm.ElevatorSparkMax;
import org.sciborgs1155.robot.subsystems.arm.JointIOSim;
import org.sciborgs1155.robot.subsystems.arm.WristSparkMax;
import org.sciborgs1155.robot.util.Visualizer;
import org.sciborgs1155.robot.util.placement.PlacementCache;
import org.sciborgs1155.robot.util.placement.PlacementState;
import org.sciborgs1155.robot.util.placement.PlacementTrajectory;

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

  public interface JointIO extends Sendable, AutoCloseable {
    public Rotation2d getRelativeAngle();

    public State getCurrentState();

    public State getDesiredState();

    public void updateDesiredState(State state);

    public void setBaseAngle(Rotation2d baseAngle);

    public Rotation2d getBaseAngle();

    public boolean isFailing();

    public double getVoltage();

    public default Rotation2d getAbsoluteAngle() {
      return getBaseAngle().plus(getRelativeAngle());
    }

    @Override
    default void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("current angle", () -> getCurrentState().position, null);
      builder.addDoubleProperty("current angular velocity", () -> getCurrentState().velocity, null);
      builder.addDoubleProperty("target angle", () -> getDesiredState().position, null);
      builder.addDoubleProperty("target angular velocity", () -> getDesiredState().velocity, null);
      builder.addDoubleProperty("base angle", () -> getBaseAngle().getRadians(), null);
      builder.addDoubleProperty("absolute angle", () -> getAbsoluteAngle().getRadians(), null);
      builder.addDoubleProperty("applied voltage", this::getVoltage, null);
    }
  }

  public interface ElevatorIO extends Sendable, AutoCloseable {
    public double getHeight();

    public State getState();

    public State getDesiredState();

    public void updateDesiredState(State state);

    public boolean isFailing();

    public double getVoltage();

    @Override
    default void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("position", () -> getState().position, null);
      builder.addDoubleProperty("velocity", () -> getState().velocity, null);
      builder.addDoubleProperty("target position", () -> getDesiredState().position, null);
      builder.addDoubleProperty("target velocity", () -> getDesiredState().velocity, null);
      builder.addBooleanProperty("failing", this::isFailing, null);
      builder.addDoubleProperty("applied voltage", this::getVoltage, null);
    }
  }

  @Log private final ElevatorIO elevator;
  @Log private final JointIO elbow;
  @Log private final JointIO wrist;

  private final Map<Integer, PlacementTrajectory> trajectories = PlacementCache.loadTrajectories();

  private final Visualizer positionVisualizer;
  private final Visualizer setpointVisualizer;

  @Log private boolean stopped = false;
  // 'tis but a scratch (see: match 42) (the elbow is unplugged)
  @Log private boolean butAScratch = false;
  @Log private boolean wristLimp = false;

  public Arm(Visualizer positionVisualizer, Visualizer setpointVisualizer) {

    if (Robot.isReal()) {
      elevator = new ElevatorSparkMax();
      elbow = new ElbowSparkMax(Elbow.PID, Elbow.FF);
      wrist = new WristSparkMax(Wrist.PID, Wrist.FF);
    } else {
      elevator = new ElevatorIOSim(Elevator.PID, Elevator.FF, Elevator.CONFIG, true);
      elbow = new JointIOSim(Elbow.PID, Elbow.FF, Elbow.CONFIG, true);
      wrist = new JointIOSim(Wrist.PID, Wrist.FF, Wrist.CONFIG, false);
    }

    this.positionVisualizer = positionVisualizer;
    this.setpointVisualizer = setpointVisualizer;
  }

  /** Returns the current position of the placement mechanisms */
  public PlacementState getState() {
    return new PlacementState(elevator.getHeight(), elbow.getRelativeAngle(), wrist.getRelativeAngle());
  }

  /** Returns the current setpoint of the placement mechanisms */
  public PlacementState getSetpoint() {
    return PlacementState.fromRelative(elevator.getDesiredState().position, elbow.getDesiredState().position, wrist.getDesiredState().position);
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
                    () -> arm.allowPassOver() || goal.get().side() == state().side()))
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
  private Command safeFollowProfile(Supplier<PlacementState> goal) {
    return Commands.either(
            followProfile(passOver(goal.side())),
            Commands.none(),
            () -> goal.side() != state().side())
        .andThen(followProfile(goal))
        .withName("safe follow profile");
  }
  

  /** Sets the position setpoints for the elbow and wrist, in radians */
  private CommandBase setSetpoints(Supplier<PlacementState> setpoint) {
    return runOnce(() -> {
      elevator.updateDesiredState(new State(setpoint.get().elevatorHeight(), 0));
      elbow.updateDesiredState(new State(setpoint.get().elbowAngle().getRadians(), 0));
      wrist.updateDesiredState(new State(setpoint.get().wristAngle().getRadians(), 0));
    }).withName("set setpoints");
  }

  /** Follows a {@link TrapezoidProfile} for each joint's relative position */
  private CommandBase followProfile(Supplier<PlacementState> goal) {
    return new DeferredCommand(
            () ->
                Commands.parallel(
                    new TrapezoidProfileCommand(
                        new TrapezoidProfile(
                            Elevator.CONSTRAINTS,
                            new State(goal.get().elevatorHeight(), 0),
                            elevator.getState()),
                        elevator::updateDesiredState),
                    new TrapezoidProfileCommand(
                        new TrapezoidProfile(
                            Elbow.CONSTRAINTS,
                            new State(goal.get().elbowAngle().getRadians(), 0),
                            elbow.getCurrentState()),
                        elbow::updateDesiredState),
                    new TrapezoidProfileCommand(
                        new TrapezoidProfile(
                            Wrist.CONSTRAINTS,
                            new State(goal.get().wristAngle().getRadians(), 0),
                            wrist.getCurrentState()),
                        wrist::updateDesiredState)),
            this)
        .withName("following profile");
  }

  /** Follows a {@link Trajectory} for each joint's relative position */
  private CommandBase followTrajectory(Supplier<PlacementTrajectory> trajectory) {
    return Commands.parallel(
            trajectory.get().elevator().follow(elevator::updateDesiredState),
            trajectory.get().elbow().follow(elbow::updateDesiredState),
            trajectory.get().wrist().follow(wrist::updateDesiredState, this))
        .withName("following trajectory");
  }

  public Command setStopped(boolean stopped) {
    return runOnce(() -> this.stopped = stopped);
  }

  @Override
  public void periodic() {
    // double elbowFB =
    //     elbowFeedback.calculate(getElbowPosition().getRadians(), elbowSetpoint.position());
    // double elbowFF =
    //     elbowFeedforward.calculate(
    //         elbowSetpoint.position(), elbowSetpoint.velocity(), elbowSetpoint.acceleration());

    // elbow.setVoltage(stopped ? 0 : elbowFB + elbowFF);

    // // wrist feedback is calculated using an absolute angle setpoint, rather than a relative one
    // // this means the extra voltage calculated to cancel out gravity is kG * cos(θ + ϕ), where θ
    // is
    // // the elbow setpoint and ϕ is the wrist setpoint
    // // the elbow angle is used as a setpoint instead of current position because we're using
    // // trajectories, which means setpoints are achievable states, rather than goals
    // double wristFB =
    //     wristFeedback.calculate(getRelativeWristPosition().getRadians(),
    // wristSetpoint.position());
    // double wristFF =
    //     wristFeedforward.calculate(
    //         wristSetpoint.position() + elbowSetpoint.position(),
    //         wristSetpoint.velocity(),
    //         wristSetpoint.acceleration());

    // wrist.setVoltage(stopped || wristLimp ? 0 : wristFB + wristFF);

    positionVisualizer.setArmAngles(getElbowPosition(), getRelativeWristPosition());
    setpointVisualizer.setArmAngles(
        Rotation2d.fromRadians(elbowFeedback.getSetpoint()),
        Rotation2d.fromRadians(wristFeedback.getSetpoint()));

    // SAFETY CHECKS
    wristLimp =
        wristEncoder.getPosition() == 0
            && wristEncoder.getVelocity() == 0
            && wristSetpoint.position() != 0
            && Robot.isReal();
    butAScratch =
        elbowEncoder.getDistance() == 0 // no position reading
            && elbowEncoder.getRate() == 0 // no velocity reading
            && elbowSetpoint.position() != Elbow.OFFSET // elbow is not going to 0
            && elbow.getAppliedOutput() != 0; // elbow is trying to move
  }

  @Override
  public void close() {
    elevator.close();
    elbow.close();
    wrist.close();
  }
}
