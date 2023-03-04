package org.sciborgs1155.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Arrays;
import org.sciborgs1155.lib.BalanceFeedforward;
import org.sciborgs1155.robot.Constants.Auto;
import org.sciborgs1155.robot.Constants.BalanceConstants;
import org.sciborgs1155.robot.subsystems.Drive;

public final class Balance {
  private static BalanceFeedforward ff =
      new BalanceFeedforward(
          BalanceConstants.KS, BalanceConstants.KV, BalanceConstants.KA, BalanceConstants.KG);

  private static ProfiledPIDController pid =
      new ProfiledPIDController(
          Auto.Cartesian.kP, Auto.MAX_SPEED, Auto.MAX_ACCEL, BalanceConstants.CONSTRAINTS);

  private static PIDController controller =
      new PIDController(BalanceConstants.KP, BalanceConstants.KI, BalanceConstants.KD);

  static {
    Shuffleboard.getTab("Balance").add("ff", ff);
    Shuffleboard.getTab("Balance").add("ppid", pid);
    Shuffleboard.getTab("Balance").add("raw pid", controller);
  }

  public static CommandBase balancePID(Drive drive) {
    return Commands.run(
            () ->
                drive.drive(
                    controller.calculate(drive.getTotalIncline(), BalanceConstants.SETPOINT),
                    0,
                    0,
                    true))
        .andThen(controller::close);
  }

  //   public static CommandBase balanceFF(Drive drive) {
  //     // relies on very good odometry/pose estimation
  //     // we also cannot slip

  //     Pose3d target = new Pose3d(); // center of balance mechanism

  //     Runnable action =
  //         () -> {
  //           double out = pid.calculate(drive.getPose().getX(), target.getX()); // might not be
  // getX
  //           out += ff.calculate(drive.getTotalIncline(), pid.getSetpoint().velocity);
  //           drive.drive(out, 0, 0, true);
  //         };

  //     return Commands.run(action, drive);
  //   }

  public static CommandBase balanceFF(Drive drive) {
    Runnable action =
        () -> {
          double avgSpeed =
              Arrays.stream(drive.getModuleStates())
                      .mapToDouble(state -> state.speedMetersPerSecond)
                      .reduce(0, Double::sum)
                  / 4;
          double out = pid.calculate(drive.getTotalIncline(), 0);
          out += ff.calculate(drive.getTotalIncline(), avgSpeed);
          drive.drive(out, 0, 0, true);
        };

    return Commands.run(action, drive);
  }
}
