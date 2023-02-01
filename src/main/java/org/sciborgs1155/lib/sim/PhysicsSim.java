package org.sciborgs1155.lib.sim;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import java.util.ArrayList;

public class PhysicsSim {

  private static final PhysicsSim sim = new PhysicsSim();

  private final ArrayList<SimProfile> simProfiles = new ArrayList<SimProfile>();

  public static PhysicsSim getInstance() {
    return sim;
  }

  /** Add {@link CANSparkMax} to be simulated with an arbitrary {@link LinearSystemSim} */
  public void addSparkMax(CANSparkMax spark, LinearSystemSim<N2, N1, N2> sim) {
    simProfiles.add(new SparkMAXSimProfile(spark, sim));
  }

  /** Add {@link CANSparkMax} to be simulated with a {@link DCMotorSim} */
  public void addSparkMax(CANSparkMax spark, DCMotorSim sim) {
    simProfiles.add(new SparkMAXSimProfile(spark, sim));
  }

  /** Runs the simulator */
  public void run() {
    for (SimProfile simProfile : simProfiles) {
      simProfile.run();
    }
  }

  // /** Add {@link CANSparkMax} to be simulated with a {@link SingleJointedArmSim} */
  // public void addSparkMax(CANSparkMax spark, SingleJointedArmSim sim) {
  //   simProfiles.add(new SparkMAXSimProfile(spark, sim));
  // }

  // TODO add pre upstream single jointed arm and elevator <N2 N1 N2> sims

  /** Holds information about a simulated device. */
  abstract static class SimProfile {
    private double lastTime;

    /** Runs the simulation profile. Implemented by device-specific profiles. */
    public abstract void run();

    /** Returns the time since last call, in milliseconds. */
    protected double getPeriod() {
      double now = Timer.getFPGATimestamp();
      final double period = now - lastTime;
      lastTime = now;

      return period;
    }
  }
}
