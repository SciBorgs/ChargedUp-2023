package org.sciborgs1155.robot.subsystems.modules;

import static org.sciborgs1155.robot.Constants.SwerveModule.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import org.sciborgs1155.lib.WheelSim;
import org.sciborgs1155.lib.constants.PIDConstants;
import org.sciborgs1155.robot.Constants;

/** Class to encapsulate a rev max swerve module */
public class SimSwerveModule implements SwerveModule {

  private final WheelSim drive =
      new WheelSim(Driving.kV, Driving.kA, DCMotor.getNEO(1), Driving.CONVERSION.gearing());
  private final WheelSim turn =
      new WheelSim(Turning.kV, Turning.kA, DCMotor.getNeo550(1), Turning.CONVERSION.gearing());

  private final PIDController driveFeedback = Driving.PID.create();
  private final PIDController turnFeedback = Turning.PID.create();

  private final SimpleMotorFeedforward driveFeedforward =
      new SimpleMotorFeedforward(Driving.kS, Driving.kV, Driving.kA);

  private SwerveModuleState setpoint = new SwerveModuleState();

  public SimSwerveModule() {
    turnFeedback.enableContinuousInput(0, Turning.CONVERSION.factor());
  }

  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        drive.getAngularVelocityRadPerSec(),
        Rotation2d.fromRadians(turn.getAngularVelocityRadPerSec()));
  }

  @Override
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        drive.getAngularPositionRad(), Rotation2d.fromRadians(turn.getAngularPositionRad()));
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    setpoint =
        SwerveModuleState.optimize(
            desiredState, Rotation2d.fromRadians(turn.getAngularPositionRad()));

    final double driveFB =
        driveFeedback.calculate(drive.getAngularVelocityRadPerSec(), setpoint.speedMetersPerSecond);
    final double driveFF = driveFeedforward.calculate(setpoint.speedMetersPerSecond);

    final double turnFB =
        turnFeedback.calculate(turn.getAngularPositionRad(), setpoint.angle.getRadians());

    drive.setInputVoltage(driveFB + driveFF);
    drive.update(Constants.RATE);
    turn.setInputVoltage(turnFB);
    turn.update(Constants.RATE);
  }

  @Override
  public SwerveModuleState getDesiredState() {
    return setpoint;
  }

  @Override
  public void resetEncoders() {
    drive.setState(VecBuilder.fill(0, 0));
    turn.setState(VecBuilder.fill(0, 0));
  }

  @Override
  public void setTurnPID(PIDConstants constants) {
    constants.set(turnFeedback);
  }

  @Override
  public void setDrivePID(PIDConstants constants) {
    constants.set(driveFeedback);
  }
}
