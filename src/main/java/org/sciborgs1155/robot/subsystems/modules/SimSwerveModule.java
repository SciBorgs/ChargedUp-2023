package org.sciborgs1155.robot.subsystems.modules;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import org.sciborgs1155.lib.WheelSim;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.ModuleConstants.Driving;
import org.sciborgs1155.robot.Constants.ModuleConstants.Turning;

/** Class to encapsulate a rev max swerve module */
public class SimSwerveModule implements SwerveModule, Sendable {

  private final WheelSim drive = new WheelSim(Driving.V, Driving.A);
  private final WheelSim turn = new WheelSim(Turning.V, Turning.A);

  private final PIDController driveFeedback = new PIDController(Driving.P, Driving.I, Driving.D);
  private final PIDController turnFeedback = new PIDController(Turning.P, Turning.I, Turning.D);

  private final SimpleMotorFeedforward driveFeedforward =
      new SimpleMotorFeedforward(Driving.S, Driving.V, Driving.A);

  /**
   * Constructs a SwerveModule for rev's product.
   *
   * @param drivePort drive motor port
   * @param turnPort turning motor port
   * @param angularOffset offset from drivetrain
   */
  public SimSwerveModule() {
    // set up continuous input for turning
    turnFeedback.enableContinuousInput(Turning.MIN_INPUT, Turning.MAX_INPUT);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(drive.getVelocity(), Rotation2d.fromRadians(turn.getPosition()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        drive.getPosition(), Rotation2d.fromRadians(turn.getPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, Rotation2d.fromRadians(turn.getPosition()));

    final double driveFB = driveFeedback.calculate(drive.getVelocity(), state.speedMetersPerSecond);
    final double driveFF = driveFeedforward.calculate(state.speedMetersPerSecond);

    final double turnFB = turnFeedback.calculate(turn.getPosition(), state.angle.getRadians());

    drive.setInput(driveFB + driveFF);
    drive.update(Constants.RATE);
    turn.setInput(turnFB);
    turn.update(Constants.RATE);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    drive.reset();
    turn.reset();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("velocity", drive::getVelocity, null);
    builder.addDoubleProperty("angle", turn::getPosition, null);
  }
}
