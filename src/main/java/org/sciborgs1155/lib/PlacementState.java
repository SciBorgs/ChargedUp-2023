package org.sciborgs1155.lib;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import org.sciborgs1155.robot.Constants.Dimensions;

/** ArmState class to store relative angles for the arm. */
public record PlacementState(
        double elevatorHeight,
        Rotation2d elbowAngle,
        Rotation2d wristAngle,
        double elevatorVelocity,
        double elbowAngularVelocity,
        double wristAngularVelocity)
        implements Sendable {

    /** Represents the side of the robot the arm is on */
    public enum Side {
        FRONT,
        BACK;
    }

    /** Creates a new PlacementState with velocities of 0 */
    public PlacementState(double elevatorHeight, Rotation2d elbowAngle, Rotation2d wristAngle) {
        this(elevatorHeight, elbowAngle, wristAngle, 0, 0, 0);
    }

    /**
     * Returns a new {@link PlacementState} from angles in radians, with the wrist state relative to
     * the chassis
     */
    public static PlacementState fromAbsolute(
            double elevatorHeight, double elbowAngle, double wristAngle) {
        return new PlacementState(
                elevatorHeight,
                Rotation2d.fromRadians(elbowAngle),
                Rotation2d.fromRadians(wristAngle - elbowAngle));
    }

    /**
     * Returns a new {@link PlacementState} from angles in radians, with the wrist state relative to
     * the forearm
     */
    public static PlacementState fromRelative(
            double elevatorHeight, double elbowAngle, double wristAngle) {
        return new PlacementState(
                elevatorHeight, Rotation2d.fromRadians(elbowAngle), Rotation2d.fromRadians(wristAngle));
    }

    /** Creates a PlacementState from a 6d vector */
    public static PlacementState fromVec(Vector<N6> state) {
        return new PlacementState(
                state.get(0, 0),
                Rotation2d.fromRadians(state.get(1, 0)),
                Rotation2d.fromRadians(state.get(2, 0)),
                state.get(3, 0),
                state.get(4, 0),
                state.get(5, 0));
    }

    /** Returns a 6d vector representation of the state */
    public Vector<N6> toVec() {
        return VecBuilder.fill(
                elevatorHeight,
                elbowAngle.getRadians(),
                wristAngle.getRadians(),
                elevatorVelocity,
                elbowAngularVelocity,
                wristAngularVelocity);
    }

    /** The side of the robot the arm is on */
    public Side side() {
        return elbowAngle.getCos() > 0 ? Side.FRONT : Side.BACK;
    }

    /**
     * Returns the corresponding arm state given a target Math can be found {@link
     * https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry}
     */
    public static PlacementState fromIK(Translation3d target, double height) {
        double wristAngle =
                -Math.acos(
                        (Math.pow(target.getY(), 2)
                                        + Math.pow(target.getZ() - height, 2)
                                        - Math.pow(Dimensions.FOREARM_LENGTH, 2)
                                        - Math.pow(Dimensions.CLAW_LENGTH, 2))
                                / 2.0
                                * Dimensions.CLAW_LENGTH
                                * Dimensions.FOREARM_LENGTH);
        double elbowAngle =
                Math.atan2(target.getZ() - height, target.getY())
                        + Math.atan2(
                                Dimensions.CLAW_LENGTH * Math.sin(wristAngle),
                                Dimensions.FOREARM_LENGTH + Dimensions.CLAW_LENGTH * Math.cos(wristAngle));
        return fromAbsolute(elbowAngle, wristAngle, 0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("elevator height", this::elevatorHeight, null);
        builder.addDoubleProperty("elbow angle", () -> elbowAngle().getDegrees(), null);
        builder.addDoubleProperty("relative wrist angle", () -> wristAngle().getDegrees(), null);
        builder.addDoubleProperty(
                "absolute wrist angle", () -> elbowAngle().plus(wristAngle()).getDegrees(), null);
        builder.addDoubleProperty("elevator velocity", this::elevatorVelocity, null);
        builder.addDoubleProperty("elbow angular velocity", this::elbowAngularVelocity, null);
        builder.addDoubleProperty("wrist angular velocity", this::wristAngularVelocity, null);
    }
}
