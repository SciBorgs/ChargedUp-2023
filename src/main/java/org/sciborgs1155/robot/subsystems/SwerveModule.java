package org.sciborgs1155.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import org.sciborgs1155.robot.Constants.ModuleConstants;
import org.sciborgs1155.robot.Constants.Motors;
import org.sciborgs1155.robot.Robot;

public class SwerveModule implements Sendable {
  private final CANSparkMax driveMotor; // Regular Neo
  private final CANSparkMax turnMotor; // Neo 550

  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turningEncoder;

  private final SparkMaxPIDController driveFeedback;
  private final SparkMaxPIDController turnFeedback;

  private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0, 0);

  private final double angularOffset;

  /**
   * Constructs a SwerveModule for rev's product.
   *
   * @param drivePort drive motor port
   * @param turnPort turning motor port
   * @param angularOffset offset from drivetrain
   */
  public SwerveModule(int drivePort, int turnPort, double angularOffset) {
    driveMotor = Motors.moduleDriveConfig.buildCanSparkMax(MotorType.kBrushless, drivePort);
    turnMotor = Motors.moduleTurnConfig.buildCanSparkMax(MotorType.kBrushless, turnPort);

    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
    driveFeedback = driveMotor.getPIDController();
    turnFeedback = turnMotor.getPIDController();
    driveFeedback.setFeedbackDevice(driveEncoder);
    turnFeedback.setFeedbackDevice(turningEncoder);

    // set up continuous input for turning
    turnFeedback.setPositionPIDWrappingEnabled(true);
    turnFeedback.setPositionPIDWrappingMinInput(-Math.PI);
    turnFeedback.setPositionPIDWrappingMaxInput(Math.PI);

    // set up smart motion constants?
    turnFeedback.setSmartMotionMaxAccel(
        ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared, 0);
    turnFeedback.setSmartMotionMaxVelocity(
        ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond, 0);

    // configure constants
    driveFeedback.setP(ModuleConstants.kPModuleDriveController);
    turnFeedback.setP(ModuleConstants.kPModuleTurningController);

    // TODO proper conversion

    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderDistancePerPulse);
    turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderDistancePerPulse);

    // burning to flash again (already done in motor config, there's probably a nicer way)
    driveMotor.burnFlash();
    turnMotor.burnFlash();

    // add motors to rev physics sim
    if (Robot.isSimulation()) {
      REVPhysicsSim.getInstance().addSparkMax(driveMotor, DCMotor.getNEO(1));
      REVPhysicsSim.getInstance().addSparkMax(turnMotor, DCMotor.getNeo550(1));
    }

    this.angularOffset = angularOffset;
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
    SwerveModuleState state =
        SwerveModuleState.optimize(correctedDesiredState, new Rotation2d(turningEncoder.getPosition()));

    // Calculate the drive output from the drive PID controller.
    driveFeedback.setReference(
        state.speedMetersPerSecond,
        ControlType.kVelocity,
        0,
        driveFeedforward.calculate(state.speedMetersPerSecond));
    turnFeedback.setReference(
        state.angle.getRadians(),
        ControlType.kSmartMotion); // change to kPosition if things get weird
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    driveEncoder.setPosition(0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("velocity", driveEncoder::getVelocity, null);
    builder.addDoubleProperty("angle", turningEncoder::getPosition, null);
  }
}
