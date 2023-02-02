package org.sciborgs1155.lib.sim;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import org.sciborgs1155.lib.sim.PhysicsSim.SimProfile;

public class SparkMAXSimProfile extends SimProfile {

  private final CANSparkMax spark;
  private final EncoderSim encoder;
  private final LinearSystemSim<N2, N1, N2> system;

  public SparkMAXSimProfile(CANSparkMax spark, LinearSystemSim<N2, N1, N2> system) {
    this.spark = spark;
    this.encoder = new EncoderSim(spark.getDeviceId());
    this.system = system;
  }

  @Override
  public void run() {
    double out = spark.getAppliedOutput();
    system.setInput(out);
    system.update(getPeriod());
    encoder.setPosition(system.getOutput(0));
    encoder.setVelocity(system.getOutput(1));
  }
}
