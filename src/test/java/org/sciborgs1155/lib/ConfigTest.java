package org.sciborgs1155.lib;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.revrobotics.CANSparkMax.IdleMode;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.lib.constants.Conversion;
import org.sciborgs1155.lib.constants.Conversion.PulsesPerRev;
import org.sciborgs1155.lib.constants.Conversion.Units;

public class ConfigTest {

  // REV MAX SWERVE DEFAULT CONSTANTS
  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the
    // bevel pinion
    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor =
        (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor =
        ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor =
        (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput =
        kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  @Test
  void conversion() {
    double radius = 0.0181864;
    double conversion = 2.0 * Math.PI * radius; // m
    double factor = conversion / 2048.0; // throughbore cpr;

    Conversion ratio =
        Conversion.base()
            .multiplyRadius(0.0181864)
            .withUnits(Units.RADIANS)
            .withPulsesPerRev(PulsesPerRev.REV_THROUGHBORE);

    assert factor == ratio.factor();
  }

  @Test
  void rev() {
    var CONVERSION =
        Conversion.base()
            .multiplyRadius(0.0381)
            .withUnits(Conversion.Units.RADIANS)
            .divideGearing(45.0)
            .divideGearing(22.0)
            .multiplyGearing(15.0)
            .multiplyGearing(14.0); // pinion teeth

    System.out.println("OURS: " + CONVERSION.factor());
    System.out.println("THEIRS:" + ModuleConstants.kDrivingEncoderPositionFactor);
    assertEquals(CONVERSION.factor(), ModuleConstants.kDrivingEncoderPositionFactor, 0.001);
  }
}
