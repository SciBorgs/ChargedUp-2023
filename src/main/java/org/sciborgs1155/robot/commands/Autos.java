package org.sciborgs1155.robot.commands;

import org.sciborgs1155.robot.subsystems.Arm;
import org.sciborgs1155.robot.subsystems.Drive;
import org.sciborgs1155.robot.subsystems.Elevator;
import org.sciborgs1155.robot.subsystems.Intake;
import org.sciborgs1155.robot.util.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Autos implements Loggable {
    @Log private final SendableChooser<Command> autoChooser;

    private final Drive drive;
    private final Arm arm;
    private final Elevator elevator;
    private final Vision vision;
    private final Intake intake;

    public Autos(Drive drive, Arm arm, Elevator elevator, Vision vision, Intake intake) {
        this.drive = drive;
        this.arm = arm;
        this.vision = vision;
        this.elevator = elevator;
        this.intake = intake;

        autoChooser = new SendableChooser<Command>();
        autoChooser.setDefaultOption("simpleDrive", simpleDriveAuto());
    }

    private final Command simpleDriveAuto() {
        return drive.driveToPose(new Pose2d(1, 5, Rotation2d.fromDegrees(0))).andThen(
               drive.driveToPose(new Pose2d(1, 1, Rotation2d.fromDegrees(0))));
    }

    public Command get() { return autoChooser.getSelected(); }
}
