package org.sciborgs1155.lib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.lib.sim.PhysicsSim;

public class PhysicsSimTest {

  @Test
  void distance() throws InterruptedException {
    CANSparkMax simMotor = new CANSparkMax(0, MotorType.kBrushless);
    RelativeEncoder encoder = simMotor.getEncoder();

    PhysicsSim.getInstance().addSparkMax(simMotor, new WheelSim(1, 1));
    PhysicsSim.getInstance().run();
    assert encoder.getPosition() == 0;
    simMotor.setVoltage(4);
    Thread.sleep(20);
    PhysicsSim.getInstance().run();
    System.out.println("sim motor position: " + encoder.getPosition());
    assert encoder.getPosition() > 0;
  }

  @Test
  void velocity() throws InterruptedException {
    CANSparkMax simMotor = new CANSparkMax(1, MotorType.kBrushless);
    RelativeEncoder encoder = simMotor.getEncoder();

    PhysicsSim.getInstance().addSparkMax(simMotor, new WheelSim(1, 1));
    PhysicsSim.getInstance().run();
    assert encoder.getVelocity() == 0;
    simMotor.setVoltage(4);
    Thread.sleep(20);
    PhysicsSim.getInstance().run();
    System.out.println("sim motor velocity: " + encoder.getVelocity());
    assert encoder.getVelocity() > 0;
  }

  @Test
  void controlTypes() throws InterruptedException {
    CANSparkMax simMotor = new CANSparkMax(2, MotorType.kBrushless);
    RelativeEncoder encoder = simMotor.getEncoder();

    PhysicsSim.getInstance().addSparkMax(simMotor, new WheelSim(1, 1));
    simMotor.set(0.3333);
    Thread.sleep(20);
    PhysicsSim.getInstance().run();
    System.out.println("sim motor velocity: " + encoder.getVelocity());
    assert encoder.getVelocity() > 0;
  }
}
