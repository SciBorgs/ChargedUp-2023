package org.sciborgs1155.robot.subsystems.modules;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import org.sciborgs1155.lib.FunctionRegistry;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.ModuleConstants;
import org.sciborgs1155.robot.Constants.ModuleConstants.Driving;
import org.sciborgs1155.robot.Constants.ModuleConstants.Turning;
import org.sciborgs1155.robot.Constants.Motors;

/** Class to encapsulate a rev max swerve module */
public class RevModule implements SwerveModule, Sendable {
  private final CANSparkMax driveMotor; // Regular Neo
  private final CANSparkMax turnMotor; // Neo 550

  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turningEncoder;

  private final PIDController driveFeedback =
      new PIDController(Driving.P, Driving.I, Driving.D, Constants.CONTROLLER_RATE);
  private final ProfiledPIDController turnFeedback =
      new ProfiledPIDController(
          Turning.P, Turning.I, Turning.D, Turning.CONSTRAINTS, Constants.CONTROLLER_RATE);

  private final SimpleMotorFeedforward driveFeedforward =
      new SimpleMotorFeedforward(Driving.S, Driving.V, Driving.A);
  private final SimpleMotorFeedforward turnFeedforward =
      new SimpleMotorFeedforward(Turning.S, Turning.V, Turning.A);

  private final double angularOffset;

  private SwerveModuleState setpoint = new SwerveModuleState();

  /**
   * Constructs a SwerveModule for rev's product.
   *
   * @param drivePort drive motor port
   * @param turnPort turning motor port
   * @param angularOffset offset from drivetrain
   */
  public RevModule(int drivePort, int turnPort, double angularOffset) {
    driveMotor = Motors.moduleDriveConfig.buildCanSparkMax(MotorType.kBrushless, drivePort);
    turnMotor = Motors.moduleTurnConfig.buildCanSparkMax(MotorType.kBrushless, turnPort);

    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);

    turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

    // encoder ratios
    driveEncoder.setPositionConversionFactor(Driving.ENCODER_POSITION_FACTOR);
    driveEncoder.setVelocityConversionFactor(Driving.ENCODER_VELOCITY_FACTOR);
    turningEncoder.setPositionConversionFactor(Turning.ENCODER_POSITION_FACTOR);
    turningEncoder.setVelocityConversionFactor(Turning.ENCODER_VELOCITY_FACTOR);

    // set up continuous input for turning
    turnFeedback.enableContinuousInput(Turning.MIN_INPUT, Turning.MAX_INPUT);

    driveEncoder.setPosition(0);

    this.angularOffset = angularOffset;

    // burning to flash again (already done in motor config, there's probably a nicer way)
    driveMotor.burnFlash();
    turnMotor.burnFlash();

    // add update method to periodic
    FunctionRegistry.getInstance().add(this::update);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // won't work until units are correct
    return new SwerveModuleState(
        driveEncoder.getVelocity(), new Rotation2d(turningEncoder.getPosition() - angularOffset));
  }

  public SwerveModulePosition getPosition() {
    // won't work until units are correct
    return new SwerveModulePosition(
        driveEncoder.getPosition(), new Rotation2d(turningEncoder.getPosition() - angularOffset));
  }

  /** run controllers, set motors */
  private void update() {
    final double driveFB =
        driveFeedback.calculate(driveEncoder.getVelocity(), setpoint.speedMetersPerSecond);
    final double driveFF = driveFeedforward.calculate(setpoint.speedMetersPerSecond);

    final double turnFB =
        turnFeedback.calculate(turningEncoder.getPosition(), setpoint.angle.getRadians());
    final double turnFF = turnFeedforward.calculate(turnFeedback.getSetpoint().velocity);

    // Calculate the drive output from the drive PID controller.
    driveMotor.setVoltage(driveFB + driveFF);
    turnMotor.setVoltage(turnFB + turnFF);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(angularOffset));
    // Optimize the reference state to avoid spinning further than 90 degrees
    setpoint =
        SwerveModuleState.optimize(
            correctedDesiredState, new Rotation2d(turningEncoder.getPosition()));
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    driveEncoder.setPosition(0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("velocity", driveEncoder::getVelocity, null);
    builder.addDoubleProperty("angle", turningEncoder::getPosition, null);
    driveFeedback.initSendable(builder);
    turnFeedback.initSendable(builder);
  }
}