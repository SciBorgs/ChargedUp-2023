package org.sciborgs1155.robot.commands;

import java.util.ArrayList;
import java.util.List;

import org.sciborgs1155.lib.Vision;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.subsystems.Drive;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class ScoringAlignment {

    public Command driveToPoint(Drive drive, PathPoint point) {
        PathConstraints constraints = new PathConstraints(Constants.Auto.MAX_SPEED, Constants.Auto.MAX_ACCEL);
        Pose2d currentPose = drive.getPose();
        PathPoint currentPoint = new PathPoint(currentPose.getTranslation(), currentPose.getRotation(), currentPose.getRotation());
        List<PathPoint> points = new ArrayList<PathPoint>(List.of(currentPoint, point));
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(constraints, points);
        return drive.follow(trajectory, false, false);
    }
    
    //** alignment using odometry */
    public Command odometryAlign(Drive drive, PathPoint scoringPoint) {
        return driveToPoint(drive, scoringPoint);
    }

    //** vision alignment (currently only works for aprilTags); still relient on odometry */
//     public Command visionAlign(Drive drive, Vision vision) {
//     }
}
