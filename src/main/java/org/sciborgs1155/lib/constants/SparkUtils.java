package org.sciborgs1155.lib.constants;

/** Add your docs here. */
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.function.Consumer;

/** Utility class that creates and configures CANSparkMax motor controllers */
public class SparkUtils {

  private static final ArrayList<CANSparkMax> sparks = new ArrayList<>();

  /**
   * Creates and configures a CANSparkMax.
   *
   * @param port
   * @param type
   * @param config
   * @return The configured CANSparkMax
   */
  public static CANSparkMax create(int port, MotorType type, Consumer<CANSparkMax> config) {
    CANSparkMax spark = new CANSparkMax(port, type);
    spark.restoreFactoryDefaults();
    config.accept(spark);
    sparks.add(spark);
    return spark;
  }

  /**
   * Burn to flash all motor configs at once, accounting for CAN bus delay. Use after fully
   * configuring motors.
   *
   * @param sparks
   */
  public static void safeBurnFlash() {
    Timer.delay(0.2);
    for (CANSparkMax spark : sparks) {
      spark.burnFlash();
      Timer.delay(0.025);
    }
    Timer.delay(0.2);
  }

  public static void disableFrames(CANSparkMax spark, int... frames) {
    for (int frame : frames) {
      spark.setPeriodicFramePeriod(PeriodicFrame.fromId(frame), 65535);
    }
  }
}
