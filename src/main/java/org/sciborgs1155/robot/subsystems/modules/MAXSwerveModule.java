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
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import org.sciborgs1155.robot.Constants.Motors;

/** Class to encapsulate a rev max swerve module */
public class MAXSwerveModule implements SwerveModule, Sendable {
  private final CANSparkMax driveMotor; // Regular Neo
  private final CANSparkMax turnMotor; // Neo 550

  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turningEncoder;

  private final SparkMaxPIDController driveFeedback;
  private final SparkMaxPIDController turnFeedback;

  private final SimpleMotorFeedforward driveFeedforward =
      new SimpleMotorFeedforward(Driving.kS, Driving.kV, Driving.kA);

  private final Rotation2d angularOffset;

  private SwerveModuleState setpoint;

  /**
   * Constructs a SwerveModule for rev's MAX Swerve.
   *
   * @param drivePort drive motor port
   * @param turnPort turning motor port
   * @param angularOffset offset from drivetrain
   */
  public MAXSwerveModule(int drivePort, int turnPort, double angularOffset) {
    driveMotor = Motors.DRIVE.buildCanSparkMax(MotorType.kBrushless, drivePort);
    turnMotor = Motors.TURN.buildCanSparkMax(MotorType.kBrushless, turnPort);

    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
    driveFeedback = driveMotor.getPIDController();
    turnFeedback = turnMotor.getPIDController();
    driveFeedback.setFeedbackDevice(driveEncoder);
    turnFeedback.setFeedbackDevice(turningEncoder);

    turningEncoder.setInverted(Turning.ENCODER_INVERTED);

    // encoder ratios
    driveEncoder.setPositionConversionFactor(Driving.ENCODER_POSITION_FACTOR);
    driveEncoder.setVelocityConversionFactor(Driving.ENCODER_VELOCITY_FACTOR);

    driveFeedback.setP(Driving.kP);
    driveFeedback.setI(Driving.kI);
    driveFeedback.setD(Driving.kD);

    turningEncoder.setPositionConversionFactor(Turning.ENCODER_POSITION_FACTOR);
    turningEncoder.setVelocityConversionFactor(Turning.ENCODER_VELOCITY_FACTOR);

    // set up continuous input for turning
    turnFeedback.setPositionPIDWrappingEnabled(true);
    turnFeedback.setPositionPIDWrappingMinInput(Turning.MIN_INPUT);
    turnFeedback.setPositionPIDWrappingMaxInput(Turning.MAX_INPUT);

    turnFeedback.setP(Turning.kP);
    turnFeedback.setI(Turning.kI);
    turnFeedback.setD(Turning.kD);

    driveMotor.burnFlash();
    turnMotor.burnFlash();

    driveEncoder.setPosition(0);
    this.angularOffset = Rotation2d.fromRadians(angularOffset);
    setpoint = new SwerveModuleState(0, Rotation2d.fromRadians(turningEncoder.getPosition()));
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveEncoder.getVelocity(),
        Rotation2d.fromRadians(turningEncoder.getPosition()).minus(angularOffset));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveEncoder.getPosition(),
        Rotation2d.fromRadians(turningEncoder.getPosition()).minus(angularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
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

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    driveEncoder.setPosition(0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("current velocity", driveEncoder::getVelocity, null);
    // builder.addDoubleProperty("current angle", () -> this.getPosition().angle.getRadians(),
    // null);
    builder.addDoubleProperty("current angle", turningEncoder::getPosition, null);
    builder.addDoubleProperty("target angle", () -> setpoint.angle.getRadians(), null);
    builder.addDoubleProperty("target velocity", () -> setpoint.speedMetersPerSecond, null);
  }
}