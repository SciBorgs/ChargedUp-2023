package org.sciborgs1155.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.subsystems.Drive;
import org.sciborgs1155.robot.subsystems.Intake;
import org.sciborgs1155.robot.util.PlacementState;
import org.sciborgs1155.robot.util.Vision;

public class Scoring {
  private final Intake intake;
  private final Drive drive;
  private final Placement placement;
  private final Vision vision;

  public Scoring(Drive drive, Placement placement, Intake intake, Vision vision) {
    this.intake = intake;
    this.drive = drive;
    this.placement = placement;
    this.vision = vision;
  }

  public Command score(GamePiece gamePiece, ScoringHeight height, Side side) {
    if (height == ScoringHeight.HIGH && side == Side.BACK) {
      throw new RuntimeException("cannot score high in the back");
    }
    return placement
        .toState(scoringState(gamePiece, height, side))
        .andThen(intake.start(true))
        .andThen(Commands.waitSeconds(3))
        .andThen(intake.stop());
  }

  // TODO make it take gamePiece into account
  public Command odometryAlign(Side side, Color color) {
    return drive.driveToPose(closestScoringPoint(side, color));
  }

  public Command shiftScoringNode(Side side, Color color, Direction direction) {
    return drive.driveToPose(shiftScoringTarget(side, color, direction));
  }

  // TODO msomebody pleasse make this look presentable
  // Works on blue, seems not to work on red side???
  // (sth to do with fact pv doesn't recognize middle tag as bestTarget for some reason (goofy))
  public Pose2d shiftScoringTarget(Side side, Color color, Direction direction) {
    ArrayList<Translation2d> scoringPoints =
        new ArrayList<>(List.copyOf(Constants.Field.SCORING_TAGS.values()));
    ArrayList<Translation2d> disposable =
        new ArrayList<>(List.copyOf(Constants.Field.SCORING_TAGS.values()));

    PhotonTrackedTarget[] tags = vision.getBestTag();
    PhotonTrackedTarget closestTag = vision.compareTags(tags);

    var tagPose = vision.getTagPose(closestTag).get();
    Translation2d tagPoseTrans = tagPose.toPose2d().getTranslation().nearest(disposable);
    Translation2d robotPos =
        tagPose.transformBy(closestTag.getBestCameraToTarget()).toPose2d().getTranslation();

    Translation2d closer = new Translation2d();
    Translation2d farthest = new Translation2d();
    Translation2d robotTrans = new Translation2d();

    double inaccuracyThreshold = 5;
    boolean secondTag = false;

    disposable.remove(tagPoseTrans);

    if (robotPos.getDistance(tagPoseTrans.nearest(disposable)) < inaccuracyThreshold) {
      closer = tagPoseTrans.nearest(disposable);
      disposable.remove(closer);
      secondTag = true;
    }

    if (robotPos.getDistance(tagPoseTrans.nearest(disposable)) < inaccuracyThreshold && secondTag) {
      farthest = tagPoseTrans.nearest(disposable);
    }

    int closest = scoringPoints.indexOf(tagPoseTrans);

    if (closest == 1 || closest == 4) {
      switch (direction) { // 1 and 4 are middle tags
        case RIGHT:
          robotTrans = closer;
          break;
        case LEFT:
          robotTrans = farthest;
      }
    } else if (closest == 0 || closest == 3) { // 0 and 3 are tags on left
      switch (direction) {
        case RIGHT:
          // TODO how to handle these? should right loop back around to opposite side or not do
          // anything?
          robotTrans = farthest;
          break;
        case LEFT:
          robotTrans = closer;
      }
    } else {
      switch (direction) { // tags on far right
        case RIGHT:
          robotTrans = closer;
          break;
        case LEFT:
          robotTrans = farthest;
      }
    }
    double rotationRad = (side.rads() + color.rads()) % (2 * Math.PI);
    return new Pose2d(robotTrans, Rotation2d.fromRadians(rotationRad));
  }
  // TODO vision alignment

  private Pose2d closestScoringPoint(Side side, Color color) {
    Collection<Translation2d> scoringPoints = Constants.Field.SCORING_POINTS.values();
    Translation2d point =
        drive
            .getPose()
            .getTranslation()
            .nearest(new ArrayList<Translation2d>(List.copyOf(scoringPoints)));
    double rotationRad = (side.rads() + color.rads()) % (2 * Math.PI);
    return new Pose2d(point, Rotation2d.fromRadians(rotationRad));
  }

  public enum Direction {
    LEFT,
    RIGHT;
  }

  public enum Side {
    BACK,
    FRONT;

    public double rads() {
      if (this == BACK) {
        return Math.PI;
      }
      return 0;
    }
  }

  public enum Color {
    RED,
    BLUE;

    public double rads() {
      if (this == RED) {
        return Math.PI;
      } else return 0;
    }
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

  public static PlacementState scoringState(GamePiece gamePiece, ScoringHeight height, Side side) {
    switch (gamePiece) {
      case CONE:
        switch (height) {
          case HIGH:
            switch (side) {
              case FRONT:
                return Constants.Positions.FRONT_HIGH_CONE;
              case BACK:
                return Constants.Positions.BACK_HIGH_CONE;
            }
          case MID:
            switch (side) {
              case FRONT:
                return Constants.Positions.FRONT_MID_CONE;
              case BACK:
                return Constants.Positions.BACK_MID_CONE;
            }
          case LOW:
            return Constants.Positions.BACK_LOW_CONE;
        }
      case CUBE:
        switch (height) {
          case HIGH:
            switch (side) {
              case FRONT:
                return Constants.Positions.FRONT_HIGH_CUBE;
              case BACK:
                return Constants.Positions.BACK_HIGH_CUBE;
            }
          case MID:
            switch (side) {
              case FRONT:
                return Constants.Positions.FRONT_MID_CUBE;
              case BACK:
                return Constants.Positions.BACK_MID_CUBE;
            }
          case LOW:
            return Constants.Positions.BACK_LOW_CUBE;
        }
    }
    throw new RuntimeException(
        "scoringState was not called on a valid arguments. \n"
            + "gamePiece: "
            + gamePiece
            + "; height: "
            + height
            + "; side: "
            + side);
  }
}
