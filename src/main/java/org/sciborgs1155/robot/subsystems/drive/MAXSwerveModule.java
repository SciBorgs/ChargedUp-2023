package org.sciborgs1155.robot.subsystems.drive;

import static org.sciborgs1155.robot.subsystems.drive.DriveConstants.SwerveModule.*;

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
import java.util.List;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.failure.FaultBuilder;
import org.sciborgs1155.lib.failure.HardwareFault;

/** Class to encapsulate a rev max swerve module */
public class MAXSwerveModule implements ModuleIO {

  private final CANSparkMax driveMotor; // Regular Neo
  private final CANSparkMax turnMotor; // Neo 550

  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turningEncoder;

  private final SparkMaxPIDController driveFeedback;
  private final SparkMaxPIDController turnFeedback;

  private final SimpleMotorFeedforward driveFeedforward =
      new SimpleMotorFeedforward(Driving.s, Driving.v, Driving.a);

  private final String name;
  private final Rotation2d angularOffset;

  private SwerveModuleState setpoint = new SwerveModuleState();

  /**
   * Constructs a SwerveModule for rev's MAX Swerve.
   *
   * @param drivePort drive motor port
   * @param turnPort turning motor port
   * @param angularOffset offset from drivetrain
   */
  public MAXSwerveModule(String name, int drivePort, int turnPort, double angularOffset) {
    this.name = name;

    driveMotor = SparkUtils.create(drivePort, MotorType.kBrushless, Driving.MOTOR_CFG);
    turnMotor = SparkUtils.create(turnPort, MotorType.kBrushless, Turning.MOTOR_CFG);

    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);

    driveFeedback = driveMotor.getPIDController();
    turnFeedback = turnMotor.getPIDController();

    driveFeedback.setFeedbackDevice(driveEncoder);
    turnFeedback.setFeedbackDevice(turningEncoder);

    turningEncoder.setInverted(Turning.ENCODER_INVERTED);

    setDrivePID(Driving.kP, Driving.kI, Driving.kD);
    setTurnPID(Turning.kP, Turning.kI, Turning.kD);

    driveEncoder.setPositionConversionFactor(Driving.CONVERSION);
    driveEncoder.setVelocityConversionFactor(Driving.CONVERSION / 60.0);
    turningEncoder.setPositionConversionFactor(Turning.CONVERSION);
    turningEncoder.setVelocityConversionFactor(Turning.CONVERSION / 60.0);

    // set up continuous input for turning
    turnFeedback.setPositionPIDWrappingEnabled(true);
    turnFeedback.setPositionPIDWrappingMinInput(0);
    turnFeedback.setPositionPIDWrappingMaxInput(Turning.CONVERSION);

    SparkUtils.disableFrames(driveMotor, 4, 5, 6);
    SparkUtils.disableFrames(turnMotor, 4, 6);

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
  public void setTurnPID(double kP, double kI, double kD) {
    turnFeedback.setP(kP);
    turnFeedback.setI(kI);
    turnFeedback.setD(kD);
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    driveFeedback.setP(kP);
    driveFeedback.setI(kI);
    driveFeedback.setD(kD);
  }

  @Override
  public List<HardwareFault> getFaults() {
    return FaultBuilder.create()
        .register(name + " drive", driveMotor)
        .register(name + " turn", turnMotor)
        .build();
  }

  @Override
  public void close() {
    driveMotor.close();
    turnMotor.close();
  }
}
