package org.sciborgs1155.robot.subsystems.modules;

import static org.sciborgs1155.robot.Constants.SwerveModule.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.sciborgs1155.lib.constants.PIDConstants;
import org.sciborgs1155.robot.Constants.Motors;

/** Class to encapsulate a rev max swerve module */
public class MAXSwerveModule implements SwerveModule {

  private final CANSparkMax driveMotor; // Regular Neo
  private final CANSparkMax turnMotor; // Neo 550

  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turningEncoder;

  private final SparkMaxPIDController driveFeedback;
  private final SparkMaxPIDController turnFeedback;

  private final SimpleMotorFeedforward driveFeedforward =
      new SimpleMotorFeedforward(Driving.kS, Driving.kV, Driving.kA);

  private final Rotation2d angularOffset;

  private SwerveModuleState setpoint = new SwerveModuleState();
  ;

  /**
   * Constructs a SwerveModule for rev's MAX Swerve.
   *
   * @param drivePort drive motor port
   * @param turnPort turning motor port
   * @param angularOffset offset from drivetrain
   */
  public MAXSwerveModule(int drivePort, int turnPort, double angularOffset) {
    driveMotor = Motors.DRIVE.build(MotorType.kBrushless, drivePort);
    turnMotor = Motors.TURN.build(MotorType.kBrushless, turnPort);

    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);

    driveFeedback = driveMotor.getPIDController();
    turnFeedback = turnMotor.getPIDController();

    driveFeedback.setFeedbackDevice(driveEncoder);
    turnFeedback.setFeedbackDevice(turningEncoder);

    // turningEncoder.setInverted(Turning.ENCODER_INVERTED);

    Driving.PID.set(driveFeedback);
    Turning.PID.set(turnFeedback);

    Driving.CONVERSION.configureSparkFactors(driveEncoder);
    Turning.CONVERSION.configureSparkFactors(turningEncoder);

    // set up continuous input for turning
    turnFeedback.setPositionPIDWrappingEnabled(true);
    turnFeedback.setPositionPIDWrappingMinInput(Turning.MIN_INPUT);
    turnFeedback.setPositionPIDWrappingMaxInput(Turning.MAX_INPUT);

    driveMotor.burnFlash();
    turnMotor.burnFlash();

    driveEncoder.setPosition(0);
    this.angularOffset = Rotation2d.fromRadians(angularOffset);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveEncoder.getVelocity(),
        Rotation2d.fromRadians(turningEncoder.getPosition()).minus(angularOffset));
  }

  @Override
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveEncoder.getPosition(),
        Rotation2d.fromRadians(turningEncoder.getPosition()).minus(angularOffset));
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(angularOffset);
    // Optimize the reference state to avoid spinning further than 90 degrees
    setpoint =
        SwerveModuleState.optimize(
            correctedDesiredState, Rotation2d.fromRadians(turningEncoder.getPosition()));

    // setpoint = desiredState;
    double driveFF = driveFeedforward.calculate(setpoint.speedMetersPerSecond);
    driveFeedback.setReference(setpoint.speedMetersPerSecond, ControlType.kVelocity, 0, driveFF);
    turnFeedback.setReference(setpoint.angle.getRadians(), ControlType.kPosition);
  }

  @Override
  public SwerveModuleState getDesiredState() {
    return setpoint;
  }

  @Override
  public void resetEncoders() {
    driveEncoder.setPosition(0);
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
