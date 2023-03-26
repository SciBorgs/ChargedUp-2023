package org.sciborgs1155.robot.util;

import java.util.ArrayList;
import java.util.List;
import org.sciborgs1155.robot.util.placement.PlacementState;

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
