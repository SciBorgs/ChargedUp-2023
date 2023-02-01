package org.sciborgs1155.robot.subsystems.modules;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import org.sciborgs1155.lib.WheelSim;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.ModuleConstants.Driving;
import org.sciborgs1155.robot.Constants.ModuleConstants.Turning;

/** Class to encapsulate a rev max swerve module */
public class SimSwerveModule implements SwerveModule, Sendable {

  private final WheelSim drive;
  private final WheelSim turn;

  private final PIDController driveFeedback;
  private final PIDController turnFeedback;

  private final SimpleMotorFeedforward driveFeedforward;

  /**
   * Constructs a SwerveModule for rev's product.
   *
   * @param drivePort drive motor port
   * @param turnPort turning motor port
   * @param angularOffset offset from drivetrain
   */
  public SimSwerveModule() {
    drive = new WheelSim(Driving.V, Driving.A, DCMotor.getNEO(1), Driving.ENCODER_VELOCITY_FACTOR);
    turn =
        new WheelSim(Turning.V, Turning.A, DCMotor.getNeo550(1), Turning.ENCODER_POSITION_FACTOR);

    driveFeedback = new PIDController(Driving.P, Driving.I, Driving.D);
    turnFeedback = new PIDController(Turning.P, Turning.I, Turning.D);

    driveFeedforward = new SimpleMotorFeedforward(Driving.S, Driving.V, Driving.A);

    // set up continuous input for turning
    turnFeedback.enableContinuousInput(Turning.MIN_INPUT, Turning.MAX_INPUT);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        drive.getAngularVelocityRadPerSec(),
        Rotation2d.fromRadians(turn.getAngularVelocityRadPerSec()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        drive.getAngularPositionRad(), Rotation2d.fromRadians(turn.getAngularPositionRad()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(
            desiredState, Rotation2d.fromRadians(turn.getAngularPositionRad()));

    final double driveFB =
        driveFeedback.calculate(drive.getAngularVelocityRadPerSec(), state.speedMetersPerSecond);
    final double driveFF = driveFeedforward.calculate(state.speedMetersPerSecond);

    final double turnFB =
        turnFeedback.calculate(turn.getAngularPositionRad(), state.angle.getRadians());

    drive.setInput(driveFB + driveFF);
    drive.update(Constants.RATE);
    turn.setInput(turnFB);
    turn.update(Constants.RATE);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    drive.setState(Matrix.mat(Nat.N2(), Nat.N1()).fill(0.0, 0.0));
    turn.setState(Matrix.mat(Nat.N2(), Nat.N1()).fill(0.0, 0.0));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("velocity", drive::getAngularVelocityRadPerSec, null);
    builder.addDoubleProperty("angle", turn::getAngularPositionRad, null);
  }
}
