package org.sciborgs1155.lib;

public class PlacementModel {
  public enum PlacementModels {
    RestrictionLess(0),
    ArmLock(1),
    FloorBlock(2),
    Restricted(3);

    private int modelType;

    private PlacementModels(int currentModel) {
      this.modelType = modelType;
    }
  }

  PlacementModels currentModel = PlacementModels.Restricted;

  public void setModel(PlacementModels desiredModel) {
    currentModel = desiredModel;
  }

  public PlacementModels getModel() {
    return currentModel;
  }
}
