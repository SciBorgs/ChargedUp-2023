package org.sciborgs1155.lib.sim;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import org.sciborgs1155.lib.sim.PhysicsSim.SimProfile;

/**
 * A simulation profile for the {@link CANSparkMax} using a {@link LinearSystem}.
 */
public class SparkMAXSimProfile extends SimProfile {

  private final CANSparkMax spark;
  private final SparkMAXSim encoder;
  private final LinearSystemSim<N2, N1, N2> system;

  public SparkMAXSimProfile(CANSparkMax spark, LinearSystemSim<N2, N1, N2> system) {
    this.spark = spark;
    this.encoder = new SparkMAXSim(spark);
    this.system = system;
  }

  @Override
  public void run() {

    double out = 0;
    switch (encoder.getControlMode()) {
      case 0: // duty cycle mode, uses get/set stored duty cycle variable in spark
        out = spark.get() * spark.getBusVoltage();
        break;
      case 2: // voltage mode, pretty straightforward
        out = spark.getAppliedOutput();
        break;
    }

    system.setInput(out);
    system.update(getPeriod());
    encoder.setPosition(system.getOutput(0));
    encoder.setVelocity(system.getOutput(1));
  }
}
