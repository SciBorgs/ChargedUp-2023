package org.sciborgs1155.robot.commands;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import org.sciborgs1155.lib.BalanceFeedForward;
import org.sciborgs1155.robot.Constants.Auto;
import org.sciborgs1155.robot.Constants.BalanceConstants;
import org.sciborgs1155.robot.subsystems.Drive;

public final class Balance {
  public static CommandBase balancePID(Drive drive) {
    PIDController controller =
        new PIDController(BalanceConstants.KP, BalanceConstants.KI, BalanceConstants.KD);
    return Commands.run(
            () ->
                drive.drive(
                    controller.calculate(drive.getPitch(), BalanceConstants.SETPOINT), 0, 0, true))
        .andThen(controller::close);
  }

  public static CommandBase balanceFF(Drive drive) {
    // relies on very good odometry/pose estimation
    // we also cannot slip
    BalanceFeedForward ff =
        new BalanceFeedForward(
            BalanceConstants.KS, BalanceConstants.KV, BalanceConstants.KA, BalanceConstants.KG);

    ProfiledPIDController pid =
        new ProfiledPIDController(
            Auto.Cartesian.kP, Auto.MAX_SPEED, Auto.MAX_ACCEL, BalanceConstants.CONSTRAINTS);

    Pose3d target = new Pose3d(); // center of balance mechanism

    Runnable action =
        () -> {
          double out = pid.calculate(drive.getPose().getX(), target.getX()); // might not be getX
          out += ff.calculate(drive.getPitch(), pid.getSetpoint().velocity);
          drive.drive(out, 0, 0, true);
        };

    return Commands.run(action, drive);
  }

  public static CommandBase balanceBangBang(Drive drive) {
    BangBangController balance = new BangBangController(5);
    return Commands.run(() -> drive.drive(balance.calculate(drive.getPitch(), 0), 0, 0, true));
  }
}
