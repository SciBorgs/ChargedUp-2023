package org.sciborgs1155.robot.subsystems.placement;

import java.util.ArrayList;
import java.util.List;
import org.sciborgs1155.lib.PlacementState;

public class Trajectory {
  private double totalTime = 0;
  private List<PlacementState> states = new ArrayList<PlacementState>();

  public double getTotalTime() {
    return totalTime;
  }

  public List<PlacementState> getStates() {
    return states;
  }
}
