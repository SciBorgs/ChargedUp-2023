package org.sciborgs1155.robot.commands;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Map;

import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.subsystems.Arm;
import org.sciborgs1155.robot.subsystems.Drive;
import org.sciborgs1155.robot.subsystems.Elevator;
import org.sciborgs1155.robot.subsystems.Intake;
import org.sciborgs1155.robot.util.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class Score {
    private final Intake intake;
    private final Drive drive;
    private final Placement placement;
    private final Vision vision;
    
    public Score(Drive drive, Placement placement, Intake intake, Vision vision) {
        this.intake = intake;
        this.drive = drive;
        this.placement = placement;
        this.vision = vision;
    }

    // make it take gamePiece into account (maybe need to change format of field constants)
    public final Command odometryAlign(RobotSide side, Color color) {
        return drive.driveToPose(closestScoringPoint(side, color));
    }

    private final Pose2d closestScoringPoint(RobotSide side, Color color) {
        Collection<Translation2d> scoringPoints = Constants.Field.SCORING_POINTS.values();
        Translation2d point = drive.getPose().getTranslation().nearest(new ArrayList<Translation2d>(List.copyOf(scoringPoints))); 
        // blue front: 180; blue back: 0; red front: 0; red front: 180
        double rotationDeg = 180;
        if (side == RobotSide.BACK) { rotationDeg = (rotationDeg + 180) % 360; }
        if (color == Color.RED) { rotationDeg = (rotationDeg + 180) % 360; }
        return new Pose2d(point, Rotation2d.fromDegrees(rotationDeg));
    }

    public enum RobotSide {
        BACK,
        FRONT
    }

    public enum Color {
        RED,
        BLUE
    }

    public enum GamePiece {
        CONE,
        CUBE
    }

    public enum ScoringHeight {
        HIGH,
        MID,
        LOW
    }

    
}
