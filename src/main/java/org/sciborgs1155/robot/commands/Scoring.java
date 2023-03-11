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
  private Pose2d shiftScoringTarget(Side side, Color color, Direction direction) {
    ArrayList<Translation2d> scoringPoints =
        new ArrayList<>(List.copyOf(Constants.Field.SCORING_POINTS.values()));
    PhotonTrackedTarget[] tags = vision.getBestTag();
    PhotonTrackedTarget closestID = vision.compareTags(tags);
    Translation2d closest2 = new Translation2d();
    Translation2d closest3 = new Translation2d();
    Translation2d robotTrans = new Translation2d();
    var tagPose = vision.getTagPose(closestID).get();
    Translation2d robotPos =
        tagPose.transformBy(closestID.getBestCameraToTarget()).toPose2d().getTranslation();
    // unnecessary but just in case
    Translation2d point = tagPose.getTranslation().toTranslation2d().nearest(scoringPoints);

    scoringPoints.remove(point);

    for (Translation2d tag : scoringPoints) {
      double distance = tag.getDistance(robotPos);
      if (distance < robotPos.getDistance(closest2)) {
        closest2 = tag;
      } else if (distance <= robotPos.getDistance(closest3)) {
        closest3 = tag;
      }
    }
    // relative to driver direction, works because of iteration order
    switch (direction) {
      case LEFT:
        robotTrans = closest3;
      case RIGHT:
        robotTrans = closest2;
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
