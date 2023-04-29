package org.sciborgs1155.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.sciborgs1155.lib.TestableSubsystem;

public class TestingUtil {
  /**
   * runs CommandScheduler repeatedly to fast forward subsystems and run commands
   *
   * @param ticks the number of times CommandScheduler is run
   */
  public static void fastForward(int ticks) {
    for (int i = 0; i < ticks; i++) {
      CommandScheduler.getInstance().run();
    }
  }

  /** runs CommandScheduler 200 times to fast forward subsystems and run commands */
  public static void fastForward() {
    fastForward(200);
  }

  /**
   * schedules and runs a command while disabled
   *
   * @param command
   */
  public static void run(Command command) {
    command.ignoringDisable(true).schedule();
    CommandScheduler.getInstance().run();
  }

  /**
   * schedules and runs a command while disabled
   *
   * @param command
   * @param runs the number of times CommandScheduler is run
   */
  public static void run(Command command, int runs) {
    command.ignoringDisable(true).schedule();
    fastForward(runs);
  }

  /**
   * closes subsystem and unregisters it from CommandScheduler
   *
   * @param subsystem
   */
  public static void closeSubsystem(TestableSubsystem subsystem) {
    CommandScheduler.getInstance().unregisterSubsystem(subsystem);
    subsystem.close();
  }
}
