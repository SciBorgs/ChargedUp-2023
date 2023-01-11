package org.sciborgs1155.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import org.sciborgs1155.lib.MotorConfig;
import org.sciborgs1155.robot.Constants.ModuleConstants;
import org.sciborgs1155.robot.Robot;

public class SwerveModule implements Sendable {
  private final CANSparkMax driveMotor; // Regular Neo
  private final CANSparkMax turnMotor; // Neo 550

  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turningEncoder;

  private final PIDController driveFeedback =
      new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController turnFeedback =
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurningController,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0, 0);
  private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(0, 0);

  /**
   * Constructs a SwerveModule.
   *
   * @param drivePort drive motor port
   * @param turnPort turning motor port
   * @param driveConfig MotorConfig for drive motor
   * @param turnConfig MotorConfig for turning motor
   */
  public SwerveModule(
      int drivePort, int turnPort, MotorConfig driveConfig, MotorConfig turnConfig) {
    driveMotor = driveConfig.buildCanSparkMax(MotorType.kBrushless, drivePort);
    turnMotor = turnConfig.buildCanSparkMax(MotorType.kBrushless, turnPort);

    driveEncoder = driveMotor.getEncoder();

    turningEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);

    // TODO proper conversion

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderDistancePerPulse);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderDistancePerPulse);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    turnFeedback.enableContinuousInput(-Math.PI, Math.PI);

    // add motors to rev physics sim
    if (Robot.isSimulation()) {
      REVPhysicsSim.getInstance().addSparkMax(driveMotor, DCMotor.getNEO(1));
      REVPhysicsSim.getInstance().addSparkMax(turnMotor, DCMotor.getNeo550(1));
    }
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // won't work until units are correct
    return new SwerveModuleState(
        driveEncoder.getVelocity(), new Rotation2d(turningEncoder.getPosition()));
  }

  public SwerveModulePosition getPosition() {
    // won't work until units are correct
    return new SwerveModulePosition(
        driveEncoder.getPosition(), new Rotation2d(turningEncoder.getPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(turningEncoder.getPosition()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        driveFeedback.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond)
            + driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        turnFeedback.calculate(turningEncoder.getPosition(), state.angle.getRadians())
            + turnFeedforward.calculate(turnFeedback.getSetpoint().velocity);

    // Calculate the turning motor output from the turning PID controller.
    driveMotor.setVoltage(driveOutput);
    turnMotor.setVoltage(turnOutput);
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
