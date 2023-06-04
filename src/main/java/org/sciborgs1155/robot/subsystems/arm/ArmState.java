package org.sciborgs1155.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;
import java.util.function.DoubleUnaryOperator;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.Dimensions;
import org.sciborgs1155.robot.Constants.Elbow;
import org.sciborgs1155.robot.Constants.Elevator;
import org.sciborgs1155.robot.Constants.RobotType;

/** ArmState class to store relative angles for the arm. */
public class ArmState implements Sendable {

  public static final ArmState OLD_INITIAL =
      ArmState.fromRelative(Elevator.ZERO_OFFSET, Elbow.OFFSET, Math.PI);

  public static final ArmState NEW_INITIAL =
      ArmState.fromRelative(Elevator.ZERO_OFFSET, Elbow.OFFSET, 2.4);

  public static final ArmState STOW = ArmState.fromRelative(0, 1.21834, Math.PI / 2.0);

  public static final ArmState NEW_STOW = ArmState.fromRelative(0, 0, 0);

  // LOWEST COG
  public static final ArmState OLD_SAFE =
      ArmState.fromAbsolute(Elevator.ZERO_OFFSET, Elbow.OFFSET + 0.1, Math.PI / 2);

  public static final ArmState NEW_SAFE =
      ArmState.fromAbsolute(Elevator.ZERO_OFFSET, Elbow.OFFSET + 0.1, Math.PI / 2);

  public static final ArmState PASS_TO_BACK = ArmState.fromAbsolute(0.03, Math.PI / 2.0, Math.PI);
  public static final ArmState PASS_TO_FRONT =
      ArmState.fromAbsolute(0.05, Math.PI / 2.0, Math.PI / 4.0);

  public static final ArmState OLD_FRONT_INTAKE = ArmState.fromAbsolute(0.454, -0.983, -0.055);

  public static final ArmState OLD_FRONT_SINGLE_SUBSTATION_CONE =
      ArmState.fromAbsolute(0.425006, 0.128855, -0.305);
  public static final ArmState OLD_FRONT_SINGLE_SUBSTATION_CUBE =
      ArmState.fromAbsolute(0.543571, -0.367516, 0.445646);
  public static final ArmState OLD_BACK_DOUBLE_SUBSTATION = ArmState.fromAbsolute(0, 2.8, Math.PI);

  public static final ArmState OLD_FRONT_MID_CONE =
      ArmState.fromAbsolute(0.061612, 0.493303, 0.001378);

  public static final ArmState OLD_BACK_MID_CONE = STOW; // TODO
  public static final ArmState OLD_BACK_HIGH_CONE = ArmState.fromAbsolute(0.253, 3.072, 2.5);
  // ele 0.2475
  public static final ArmState OLD_FRONT_MID_CUBE =
      ArmState.fromAbsolute(0.11362, 0.458149, 0.353288);
  public static final ArmState OLD_FRONT_HIGH_CUBE =
      ArmState.fromAbsolute(0.113502, 0.333258, 0.353208);

  public static final ArmState NEW_FRONT_MID_CUBE = ArmState.fromAbsolute(0, 0, 0);
  public static final ArmState NEW_FRONT_HIGH_CUBE = ArmState.fromAbsolute(0, 0, 0);

  public static final ArmState OLD_BACK_MID_CUBE = OLD_FRONT_MID_CUBE; // TODO
  public static final ArmState OLD_BACK_HIGH_CUBE = ArmState.fromAbsolute(0.245, 2.75, 3.17);

  public static final ArmState NEW_GROUND_CONE_INTAKE =
      ArmState.fromRelative(0.618, -0.714, -0.723);

  public static final ArmState NEW_GROUND_CUBE_INTAKE = ArmState.fromRelative(0, 0, 0);

  private double elevatorHeight;
  private Rotation2d elbowAngle;
  private Rotation2d wristAngle;

  public ArmState(double elevatorHeight, Rotation2d elbowAngle, Rotation2d wristAngle) {
    this.elevatorHeight = elevatorHeight;
    this.elbowAngle = elbowAngle;
    this.wristAngle = wristAngle;
    SmartDashboard.putData(this);
  }

  public double elevatorHeight() {
    return elevatorHeight;
  }

  public Rotation2d elbowAngle() {
    return elbowAngle;
  }

  public Rotation2d wristAngle() {
    return wristAngle;
  }

  public void setEndpointHeight(double height) {
    replaceIfPresent(
        ArmState.fromEndpoint(
            new Pose2d(getEndpoint().getX(), height, getEndpoint().getRotation())));
  }

  public void setEndpointReach(double reach) {
    replaceIfPresent(
        ArmState.fromEndpoint(
            new Pose2d(reach, getEndpoint().getY(), getEndpoint().getRotation())));
  }

  public void setEndpointRads(double rads) {
    replaceIfPresent(
        ArmState.fromEndpoint(
            new Pose2d(getEndpoint().getX(), getEndpoint().getY(), Rotation2d.fromRadians(rads))));
  }

  private void replaceIfPresent(Optional<ArmState> newState) {
    if (newState.isPresent()) {
      this.elbowAngle = newState.get().elbowAngle();
      this.elevatorHeight = newState.get().elevatorHeight();
      this.wristAngle = newState.get().wristAngle();
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("final height", () -> getEndpoint().getY(), this::setEndpointHeight);
    builder.addDoubleProperty("final reach", () -> getEndpoint().getX(), this::setEndpointReach);
    builder.addDoubleProperty(
        "final angle (rads)",
        () -> getEndpoint().getRotation().getRadians(),
        this::setEndpointRads);
    // TODO delete body below this
    builder.addDoubleProperty("elevator height", this::elevatorHeight, null);
    builder.addDoubleProperty("elbow angle (rads)", () -> elbowAngle().getRadians(), null);
    builder.addDoubleProperty("wrist angle (rads)", () -> wristAngle().getRadians(), null);
  }
  ;

  /** Represents the side of the robot the arm is on */
  public enum Side {
    FRONT,
    BACK;

    public double rads() {
      if (this == BACK) {
        return Math.PI;
      }
      return 0;
    }
  }

  /** Represents the game piece the robot is holding */
  public enum GamePiece {
    CONE,
    CUBE;
  }

  /** Represents the part of the game the robot is interacting with */
  public enum Level {
    HIGH,
    MID,
    LOW,
    SINGLE_SUBSTATION,
    DOUBLE_SUBSTATION,
  }

  /**
   * Returns a new {@link ArmState} from angles in radians, with the wrist state relative to the
   * chassis
   */
  public static ArmState fromAbsolute(double elevatorHeight, double elbowAngle, double wristAngle) {
    return new ArmState(
        elevatorHeight,
        Rotation2d.fromRadians(elbowAngle),
        Rotation2d.fromRadians(wristAngle - elbowAngle));
  }

  /**
   * Returns a new {@link ArmState} from angles in radians, with the wrist state relative to the
   * forearm
   */
  public static ArmState fromRelative(double elevatorHeight, double elbowAngle, double wristAngle) {
    return new ArmState(
        elevatorHeight, Rotation2d.fromRadians(elbowAngle), Rotation2d.fromRadians(wristAngle));
  }

  /** Creates a PlacementState from an absolute 3d array */
  public static ArmState fromArray(double[] state) {
    return fromAbsolute(state[0], state[1], state[2]);
  }

  /** Returns an array representation of the absolute state */
  public double[] toArray() {
    return new double[] {
      elevatorHeight, elbowAngle.getRadians(), wristAngle.getRadians() + elbowAngle.getRadians()
    };
  }

  /** The side of the robot the arm is on */
  public Side side() {
    return elbowAngle.getCos() > 0 ? Side.FRONT : Side.BACK;
  }

  /**
   * Returns a goal based on the side being moved to. This is used for on the fly trapezoidal
   * control.
   *
   * @param side The side of the robot being moved to.
   * @return The "passing over" goal according to the inputted side.
   */
  public static ArmState passOverToSide(Side side) {
    return side == Side.FRONT ? PASS_TO_FRONT : PASS_TO_BACK;
  }

  /** Returns a PlacementState from scoring parameters */
  public static ArmState fromOperator(Level height, GamePiece gamePiece, Side side) {
    return Constants.ROBOT_TYPE == RobotType.WHIPLASH_ROLLER
        ? fromOperatorNew(height, gamePiece, side)
        : fromOperatorOld(height, gamePiece, side);
  }

  /** Returns a PlacementState from scoring parameters based on new arm design */
  public static ArmState fromOperatorNew(Level height, GamePiece gamePiece, Side side) {
    return switch (height) {
      case LOW -> switch (gamePiece) {
        case CONE -> NEW_GROUND_CONE_INTAKE;
        case CUBE -> NEW_GROUND_CUBE_INTAKE;
      };
      case MID -> switch (gamePiece) {
        case CONE -> side == Side.FRONT ? OLD_FRONT_MID_CONE : OLD_BACK_MID_CONE;
        case CUBE -> side == Side.FRONT ? OLD_FRONT_MID_CUBE : OLD_BACK_MID_CUBE;
      };
      case HIGH -> switch (gamePiece) {
        case CONE -> OLD_BACK_HIGH_CONE;
        case CUBE -> side == Side.FRONT ? OLD_FRONT_HIGH_CUBE : OLD_BACK_HIGH_CUBE;
      };
      case SINGLE_SUBSTATION -> switch (gamePiece) {
        case CONE -> OLD_FRONT_SINGLE_SUBSTATION_CONE;
        case CUBE -> OLD_FRONT_SINGLE_SUBSTATION_CUBE;
      };
      case DOUBLE_SUBSTATION -> OLD_BACK_DOUBLE_SUBSTATION;
    };
  }

  /** Returns a PlacementState from scoring parameters based on old arm design */
  public static ArmState fromOperatorOld(Level height, GamePiece gamePiece, Side side) {
    return switch (height) {
      case LOW -> OLD_FRONT_INTAKE;
      case MID -> switch (gamePiece) {
        case CONE -> side == Side.FRONT ? OLD_FRONT_MID_CONE : OLD_BACK_MID_CONE;
        case CUBE -> side == Side.FRONT ? OLD_FRONT_MID_CUBE : OLD_BACK_MID_CUBE;
      };
      case HIGH -> switch (gamePiece) {
        case CONE -> OLD_BACK_HIGH_CONE;
        case CUBE -> side == Side.FRONT ? OLD_FRONT_HIGH_CUBE : OLD_BACK_HIGH_CUBE;
      };
      case SINGLE_SUBSTATION -> switch (gamePiece) {
        case CONE -> OLD_FRONT_SINGLE_SUBSTATION_CONE;
        case CUBE -> OLD_FRONT_SINGLE_SUBSTATION_CUBE;
      };
      case DOUBLE_SUBSTATION -> OLD_BACK_DOUBLE_SUBSTATION;
    };
  }

  /**
   * Creates a new ArmState based on the provided pose for the endpoint using inverse kinematics.
   *
   * @param endpoint A Pose2d, where translation represents the coordinates of the wrist joint and
   *     rotation represents the rotation of the wrist.
   * @return An optional, either containing the computed ArmState or None if such a state is
   *     impossible,
   */
  public static Optional<ArmState> fromEndpoint(Pose2d endpoint) {
    // find coordinate of wrist (end of forearm) relative to the elevator on top of our chassis
    var translation = endpoint.getTranslation().minus(Dimensions.BASE);

    // cos(theta) = adj / hypot
    double elbowAngle = Math.acos(translation.getX() / Dimensions.FOREARM_LENGTH);

    // fn to calculate needed elevator height from the difference between our goal and the y
    // component of our forearm
    DoubleUnaryOperator elevatorHeight =
        theta -> translation.getY() - Math.sin(theta) * Dimensions.FOREARM_LENGTH;

    // try again with the other possible elbow angle, in case we have an invalid state
    if (elevatorHeight.applyAsDouble(elbowAngle) < Elevator.MIN_HEIGHT
        || elevatorHeight.applyAsDouble(elbowAngle) > Elevator.MAX_HEIGHT) {
      elbowAngle *= -1; // reflect elbow over x-axis
    }

    // finalize elevator height
    double finalElevatorHeight = elevatorHeight.applyAsDouble(elbowAngle);

    // discard if the state is still invalid
    if (finalElevatorHeight < Elevator.MIN_HEIGHT
        || finalElevatorHeight > Elevator.MAX_HEIGHT
        || Double.isNaN(elbowAngle)) {
      return Optional.empty();
    }

    return Optional.of(
        fromAbsolute(finalElevatorHeight, elbowAngle, endpoint.getRotation().getRadians()));
  }

  /** Performs forward kinematics to return the claw's position as a {@link Pose2d} */
  public Pose2d getEndpoint() {
    return new Pose2d(
        Dimensions.BASE
            .plus(new Translation2d(0, elevatorHeight))
            .plus(new Translation2d(Dimensions.FOREARM_LENGTH, elbowAngle)),
        elbowAngle.plus(wristAngle));
  }
}
