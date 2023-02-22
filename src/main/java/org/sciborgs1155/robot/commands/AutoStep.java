package org.sciborgs1155.robot.commands;

import org.sciborgs1155.robot.commands.AutoPath.*;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public interface AutoStep extends Sendable {

    @Override
    void initSendable(SendableBuilder builder);

    public final class Score implements AutoStep {
        private final SendableChooser<GamePiece> gamePieceChooser;
        private final SendableChooser<ScoringHeight> scoringHeightChooser;
        private final Pose2d scoringPose;

        public Score(Pose2d scoringPose) {
            gamePieceChooser = new SendableChooser<GamePiece>();
            gamePieceChooser.setDefaultOption("cube", GamePiece.CUBE);
            gamePieceChooser.addOption("cone", GamePiece.CONE);

            scoringHeightChooser = new SendableChooser<ScoringHeight>();
            scoringHeightChooser.setDefaultOption("low", ScoringHeight.LOW);
            scoringHeightChooser.addOption("mid", ScoringHeight.MID);
            scoringHeightChooser.addOption("high", ScoringHeight.HIGH);
            
            this.scoringPose = scoringPose;
        }

        @Override 
        public void initSendable(SendableBuilder builder) {
            gamePieceChooser.initSendable(builder);
            scoringHeightChooser.initSendable(builder);
        }
    }
    
}
