package org.sciborgs1155.robot.util.placement;

import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.sciborgs1155.lib.trajectory.PositionTrajectory;
import org.sciborgs1155.robot.Constants.Positions;

public class PlacementCache {
  private static final String cacheFilename = "arm_trajectory_cache.json";
  private static final String cacheRequestFilename = "arm_trajectory_cache_request.json";

  /** Reads cached trajectories and returns a list of them */
  // public static List<Trajectory> loadTrajectories() {
  //   ObjectMapper mapper = new ObjectMapper();
  //   mapper.configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
  //   StoredTrajectory cache;
  //   try {
  //     cache = mapper.readValue(new File (Filesystem.getDeployDirectory(), cacheFilename),
  // StoredTrajectory.class);
  //   } catch (IOException e) {
  //     throw new RuntimeException("Failed to parse");
  //   }
  // }

  /** Generates trajectories between every stored preset */
  public static void main(String... args) throws IOException, InterruptedException {
    System.out.println("Generating...");
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
            Positions.BACK_HIGH_CUBE);

    for (var initialPos : presets) {
      for (var finalPos : presets) {
        if (!initialPos.roughlyEquals(finalPos, 0.01)) {
          generatedTrajectories.add(
              new CachedTrajectory(
                  new double[] {
                    initialPos.elevatorHeight(),
                    initialPos.elbowAngle().getRadians(),
                    initialPos.wristAngle().getRadians()
                  },
                  new double[] {
                    finalPos.elevatorHeight(),
                    finalPos.elbowAngle().getRadians(),
                    finalPos.wristAngle().getRadians()
                  },
                  new String[] {},
                  0,
                  new double[] {}));
        }
      }
    }

    ObjectMapper mapper = new ObjectMapper();
    mapper.configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
    int id = (int) (Math.random() * 10000);
    // TODO: add actual checks

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

  public PositionTrajectory getTrajectory(PlacementState start, PlacementState end) {
    return null;
  }
}
