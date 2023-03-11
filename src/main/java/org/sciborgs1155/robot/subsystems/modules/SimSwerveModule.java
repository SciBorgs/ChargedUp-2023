package org.sciborgs1155.robot.subsystems.modules;

import static org.sciborgs1155.robot.Constants.SwerveModule.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.sciborgs1155.lib.constants.PIDConstants;
import org.sciborgs1155.robot.Constants;

/** Class to encapsulate a rev max swerve module */
public class SimSwerveModule implements SwerveModule {

  private final DCMotorSim drive = Driving.FF.sim(DCMotor.getNEO(1), Driving.CONVERSION.gearing());
  private final DCMotorSim turn =
      Turning.FF.sim(DCMotor.getNeo550(1), Turning.CONVERSION.gearing());

  private final PIDController driveFeedback =
      new PIDController(Driving.PID.p(), Driving.PID.i(), Driving.PID.d());
  private final PIDController turnFeedback =
      new PIDController(Turning.PID.p(), Turning.PID.i(), Turning.PID.d());

  private final SimpleMotorFeedforward driveFeedforward =
      new SimpleMotorFeedforward(Driving.FF.s(), Driving.FF.v(), Driving.FF.a());

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
    turnFeedback.setPID(constants.p(), constants.i(), constants.d());
  }

  @Override
  public void setDrivePID(PIDConstants constants) {
    driveFeedback.setPID(constants.p(), constants.i(), constants.d());
  }
}
