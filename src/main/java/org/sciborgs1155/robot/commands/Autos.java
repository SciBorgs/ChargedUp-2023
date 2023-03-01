package org.sciborgs1155.robot.commands;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.sciborgs1155.lib.Vision;
import org.sciborgs1155.robot.subsystems.Drive;

public final class Autos implements Sendable {

  private final Drive drive;
  private final Vision vision;
  private final SendableChooser<Command> chooser;

  public Autos(Drive drive, Vision vision) {
    this.drive = drive;
    this.vision = vision;
    chooser = new SendableChooser<>();
    chooser.setDefaultOption("mobility", mobility());
    chooser.addOption("other", drive.follow("New Path", true, false));
  }

  public Command get() {
    return chooser.getSelected();
  }

  private Command mobility() {
    return Commands.run(() -> drive.drive(0.8, 0, 0, false), drive)
        .withTimeout(1)
        .andThen(() -> drive.drive(-0.8, 0, 0, false), drive)
        .withTimeout(1);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    chooser.initSendable(builder);
  }

  public Command balance() {
    double tolerance = 5;
    BangBangController balance = new BangBangController(tolerance);
    return Commands.run(() -> drive.drive(balance.calculate(drive.getPitch(), 0), 0, 0, true));
  }
}
