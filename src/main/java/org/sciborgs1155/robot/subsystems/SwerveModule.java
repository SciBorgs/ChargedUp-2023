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
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.sciborgs1155.robot.Constants.ModuleConstants;
import org.sciborgs1155.robot.Constants.ModuleConstants.Driving;
import org.sciborgs1155.robot.Constants.ModuleConstants.Turning;
import org.sciborgs1155.robot.Constants.Motors;
import org.sciborgs1155.robot.Robot;

/** Class to encapsulate a rev max swerve module */
public class SwerveModule implements Sendable {
  private final CANSparkMax driveMotor; // Regular Neo
  private final CANSparkMax turnMotor; // Neo 550

  private final FlywheelSim driveSim;
  private final FlywheelSim turnSim;
  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turningEncoder;

  private final PIDController driveFeedback = new PIDController(Driving.P, Driving.I, Driving.D);
  private final ProfiledPIDController turnFeedback =
      new ProfiledPIDController(Turning.P, Turning.I, Turning.D, Turning.CONSTRAINTS);

  private final SimpleMotorFeedforward driveFeedforward =
      new SimpleMotorFeedforward(Driving.S, Driving.V, Driving.A);
  private final SimpleMotorFeedforward turnFeedforward =
      new SimpleMotorFeedforward(Turning.S, Turning.V, Turning.A);

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

    turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

    // encoder ratios
    driveEncoder.setPositionConversionFactor(Driving.ENCODER_POSITION_FACTOR);
    driveEncoder.setVelocityConversionFactor(Driving.ENCODER_VELOCITY_FACTOR);
    turningEncoder.setPositionConversionFactor(Turning.ENCODER_POSITION_FACTOR);
    turningEncoder.setVelocityConversionFactor(Turning.ENCODER_VELOCITY_FACTOR);

    // set up continuous input for turning
    turnFeedback.enableContinuousInput(Turning.MIN_INPUT, Turning.MAX_INPUT);

    driveEncoder.setPosition(0);

    // burning to flash again (already done in motor config, there's probably a nicer way)

    driveSim = new FlywheelSim(DCMotor.getNEO(1), 1, 10);
    turnSim = new FlywheelSim(DCMotor.getNeo550(1), 1, 10);

    // add motors to rev physics sim
    if (Robot.isSimulation()) {
      driveMotor.setInverted(true);
      REVPhysicsSim.getInstance().addSparkMax(driveMotor, (float) 2.6, 900);
      REVPhysicsSim.getInstance().addSparkMax(turnMotor, (float) 0.97, 1050);
      this.angularOffset = 0;
      // turningEncoder.setZeroOffset(angularOffset);
    } else {
      this.angularOffset = angularOffset;
    }

    driveMotor.burnFlash();
    turnMotor.burnFlash();
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
        SwerveModuleState.optimize(
            correctedDesiredState, new Rotation2d(turningEncoder.getPosition()));

    final double driveFB =
        driveFeedback.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond);
    final double driveFF = driveFeedforward.calculate(state.speedMetersPerSecond);

    System.out.println(String.format("velocity: %.2f", driveEncoder.getVelocity()));
    System.out.println(String.format("position: %.2f", driveEncoder.getPosition()));
    System.out.println(String.format("rotation: %.2f", turningEncoder.getPosition()));

    final double turnFB =
        turnFeedback.calculate(turningEncoder.getPosition(), state.angle.getRadians());
    final double turnFF = turnFeedforward.calculate(turnFeedback.getSetpoint().velocity);

    // Calculate the drive output from the drive PID controller.
    driveMotor.setVoltage(driveFB + driveFF);
    turnMotor.setVoltage(turnFB + turnFF);
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
