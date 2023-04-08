package org.sciborgs1155.robot.util.placement;

import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.sciborgs1155.lib.Trajectory;
import org.sciborgs1155.robot.Constants.Positions;
import org.sciborgs1155.robot.util.placement.PlacementTrajectory.Parameters;

public class PlacementCache {
  private static final String cacheFilename = "arm_trajectory_cache.json";
  private static final String cacheRequestFilename = "arm_trajectory_cache_request.json";

  /** Reads cached trajectories and returns a list of them */
  public static Map<Integer, PlacementTrajectory> loadTrajectories() {
    ObjectMapper mapper = new ObjectMapper();
    mapper.configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
    StoredTrajectory cache;
    try {
      cache =
          mapper.readValue(
              new File(Filesystem.getDeployDirectory(), cacheFilename), StoredTrajectory.class);
    } catch (IOException e) {
      throw new RuntimeException("Failed to parse");
    }

    Map<Integer, PlacementTrajectory> trajectories = new HashMap<Integer, PlacementTrajectory>();

    for (var cachedTrajectory : cache.trajectories) {
      List<Double> elevatorStates = new ArrayList<Double>();
      List<Double> elbowStates = new ArrayList<Double>();
      List<Double> wristStates = new ArrayList<Double>();

      for (int i = 0; i <= cachedTrajectory.points().length - 3; i += 3) {
        elevatorStates.add(cachedTrajectory.points()[i]);
        elbowStates.add(cachedTrajectory.points()[i + 1]);
        wristStates.add(cachedTrajectory.points()[i + 2] - cachedTrajectory.points()[i + 1]);
      }

      Parameters params =
          new Parameters(
              PlacementState.fromArray(cachedTrajectory.initialPos),
              PlacementState.fromArray(cachedTrajectory.finalPos));

      trajectories.put(
          params.hashCode(),
          new PlacementTrajectory(
              new Trajectory(elevatorStates, cachedTrajectory.totalTime),
              new Trajectory(elbowStates, cachedTrajectory.totalTime),
              new Trajectory(wristStates, cachedTrajectory.totalTime),
              params));
    }

    return trajectories;
  }

  /** Generates trajectories between every stored preset */
  public static void main(String... args) throws IOException, InterruptedException {
    System.out.println("Generating trajectories...");
    // Generate trajectories between stored presets

    List<CachedTrajectory> generatedTrajectories = new ArrayList<CachedTrajectory>();
    var presets =
        List.of(
            Positions.INITIAL,
            Positions.STOW,
            Positions.FRONT_INTAKE,
            Positions.BACK_INTAKE,
            Positions.FRONT_SINGLE_SUBSTATION_CONE,
            Positions.FRONT_SINGLE_SUBSTATION_CUBE,
            Positions.BACK_DOUBLE_SUBSTATION,
            Positions.FRONT_MID_CONE,
            Positions.BACK_MID_CONE,
            Positions.BACK_HIGH_CONE,
            Positions.FRONT_MID_CUBE,
            Positions.FRONT_HIGH_CUBE,
            Positions.BACK_MID_CUBE,
            Positions.BACK_HIGH_CUBE,
            Positions.BALANCE);

    for (var initialPos : presets) {
      for (var finalPos : presets) {
        if (!initialPos.equals(finalPos)) {
          System.out.println("Adding initial: " + initialPos + " final: " + finalPos);
          generatedTrajectories.add(
              new CachedTrajectory(
                  initialPos.toArray(), finalPos.toArray(), new String[] {}, 0, new double[] {}));
        }
      }
    }

    ObjectMapper mapper = new ObjectMapper();
    mapper.configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
    int id = (int) (Math.random() * 10000);

    File cacheFile = Path.of(System.getProperty("java.io.tmpdir"), cacheRequestFilename).toFile();
    mapper.writeValue(cacheFile, new StoredTrajectory(id, generatedTrajectories));
  }

  public static record CachedTrajectory(
      double[] initialPos,
      double[] finalPos,
      String[] constraintOverrides,
      double totalTime,
      double[] points) {}

  public static record StoredTrajectory(int id, List<CachedTrajectory> trajectories) {}
}