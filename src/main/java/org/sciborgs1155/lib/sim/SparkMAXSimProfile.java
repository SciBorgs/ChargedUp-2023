package org.sciborgs1155.lib.sim;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import org.sciborgs1155.lib.sim.PhysicsSim.SimProfile;

public class SparkMAXSimProfile extends SimProfile {

  private final CANSparkMax spark;
  private final LinearSystemSim<N2, N1, N2> system;

  public SparkMAXSimProfile(CANSparkMax spark, LinearSystemSim<N2, N1, N2> system) {
    this.spark = spark;
    this.system = system;
  }

  @Override
  public void run() {
    double out = spark.getAppliedOutput();
    system.setInput(out);
    system.update(getPeriod());
    spark.getEncoder().setPosition(system.getOutput(0));
  }
}
