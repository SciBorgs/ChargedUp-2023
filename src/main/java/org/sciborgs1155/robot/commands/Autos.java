// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.commands;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.sciborgs1155.robot.subsystems.Drive;

public final class Autos implements Sendable {

  private final Drive drive;

  private final SendableChooser<Command> chooser;

  public Autos(Drive drive) {
    this.drive = drive;

    chooser = new SendableChooser<>();
    chooser.setDefaultOption("mobility", mobility());
    chooser.addOption("other", drive.follow("New Path"));
  }

  public Command get() {
    return chooser.getSelected();
  }

  private Command mobility() {
    return Commands.run(() -> drive.drive(0.5, 0, 0, false), drive).withTimeout(5);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    chooser.initSendable(builder);
  }
}
