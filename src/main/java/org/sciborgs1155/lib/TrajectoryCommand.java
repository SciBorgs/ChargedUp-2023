package org.sciborgs1155.lib;

import static org.sciborgs1155.lib.Trajectory.State;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Consumer;

/**
 * A command that runs a {@link Trajectory}. Useful for smoothly controlling mechanism motion.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class TrajectoryCommand extends CommandBase {
  private final Trajectory trajectory;
  private final Consumer<State> output;

  private final Timer timer = new Timer();

  /**
   * Creates a new TrajectoryCommand that will execute the given {@link Trajectory}. Output will be
   * piped to the provided consumer function.
   *
   * @param trajectory The motion profile to execute.
   * @param output The consumer for the trajectory output.
   * @param requirements The subsystems required by this command.
   */
  public TrajectoryCommand(
      Trajectory trajectory, Consumer<State> output, Subsystem... requirements) {
    this.trajectory = trajectory;
    this.output = output;
    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  @Override
  public void execute() {
    output.accept(trajectory.sample(timer.get()));
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.totalTime());
  }
}
