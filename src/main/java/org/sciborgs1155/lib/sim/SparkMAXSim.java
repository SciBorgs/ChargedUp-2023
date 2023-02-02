package org.sciborgs1155.lib.sim;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimInt;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI.SimValueInfo;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

/**
 * Sim class for CAN Encoders that uses JNI to interact with real Encoder instances. Thank you 6738!
 */
public class SparkMAXSim {
  private final SimDouble position, velocity, appliedOutput, motorCurrent;
  private final SimInt controlMode;

  /** Utility function to print the JNI values of the device */
  public static void printValues(SimDeviceSim device) {
    for (SimValueInfo val : device.enumerateValues()) {
      System.out.println(
          "Name: " + val.name + " | Handle: " + val.handle + " | Type: " + val.value);
    }
  }

  public SparkMAXSim(CANSparkMax spark) {
    this(spark.getDeviceId());
  }

  public SparkMAXSim(int deviceID) {
    SimDeviceSim device = new SimDeviceSim("SPARK MAX [" + deviceID + "]");
    // printValues(device);
    position = device.getDouble("Position");
    velocity = device.getDouble("Velocity");
    appliedOutput = device.getDouble("Applied Output");
    controlMode = device.getInt("Control Mode");
    motorCurrent = device.getDouble("Motor Current");
  }

  public void setVelocity(double v) {
    velocity.set(v);
  }

  public double getVelocity() {
    return velocity.get();
  }

  public void setPosition(double pos) {
    position.set(pos);
  }

  public double getPosition() {
    return position.get();
  }

  public double getAppliedOutput() {
    return appliedOutput.get();
  }

  public int getControlMode() {
    return controlMode.get();
  }

  public double getMotorCurrent() {
    return motorCurrent.get();
  }
}
