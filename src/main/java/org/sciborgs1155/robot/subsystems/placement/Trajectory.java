package org.sciborgs1155.robot.subsystems.placement;

import java.util.ArrayList;
import java.util.List;

public class Trajectory {
  private double totalTime = 0;
  private List<State> states = new ArrayList<State>();

  public double getTotalTime() {
    return totalTime;
  }

  public List<State> getStates() {
    return states;
  }
}
