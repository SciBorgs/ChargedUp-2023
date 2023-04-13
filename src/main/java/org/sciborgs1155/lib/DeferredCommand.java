package org.sciborgs1155.lib;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;

/**
 * Runs the given command when this command is initialized, and ends when it ends. Useful for
 * performing runtime tasks before creating a new command. If this command is interrupted, it will
 * cancel the command.
 *
 * <p>This class is provided by the NewCommands VendorDep
 *
 * @see https://github.com/wpilibsuite/allwpilib/issues/5150
 */
public class DeferredCommand extends CommandBase {
  private final Supplier<Command> m_cmdSupplier;
  private Command m_command = Commands.print("[DeferredCommand] Supplied command was null!");

  /**
   * Creates a new DeferredCommand that runs the supplied command when initialized, and ends when it
   * is no longer scheduled. Useful for lazily creating commands at runtime.
   *
   * @param supplier the command supplier
   */
  public DeferredCommand(Supplier<Command> supplier, Subsystem... requirements) {
    m_cmdSupplier = requireNonNullParam(supplier, "supplier", "DeferredCommand");
    addRequirements(requirements);
  }

  public DeferredCommand(Command command) {
    this(() -> command);
    setName("Deferred(" + command.getName() + ")");
  }

  @Override
  public void initialize() {
    var cmd = m_cmdSupplier.get();
    if (cmd != null) {
      m_command = cmd;
    }
    m_command.initialize();
  }

  @Override
  public void execute() {
    m_command.execute();
  }

  @Override
  public boolean isFinished() {
    return m_command.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    m_command.end(interrupted);
    m_command = Commands.none();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addStringProperty(
        "deferred", () -> m_command == null ? "null" : m_command.getName(), null);
  }
}
