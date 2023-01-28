package org.sciborgs1155.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import org.sciborgs1155.robot.Constants.BalanceConstants;
import org.sciborgs1155.robot.subsystems.Drivetrain;

public final class Balance {
  public static CommandBase balancePID(Drivetrain drive) {
    PIDController controller =
        new PIDController(
            BalanceConstants.BALANCE_KP, BalanceConstants.BALANCE_KI, BalanceConstants.BALANCE_KD);
    return Commands.run(() -> drive.drive(controller.calculate(drive.getPitch()), 0, 0, true));
  }
}
