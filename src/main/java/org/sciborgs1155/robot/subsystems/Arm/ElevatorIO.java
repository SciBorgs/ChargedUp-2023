package org.sciborgs1155.robot.subsystems.Arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public interface ElevatorIO extends AutoCloseable {

public State getDesiredstate();

public double getCurrentposition();

public void updateDesiredstate(double height);

public boolean isFailing();
}