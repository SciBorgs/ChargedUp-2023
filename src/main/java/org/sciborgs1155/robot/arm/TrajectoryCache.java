package org.sciborgs1155.robot.arm;

import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.sciborgs1155.lib.Trajectory;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.RobotType;

public class TrajectoryCache {

  public static record ArmTrajectory(
      Trajectory elevator, Trajectory elbow, Trajectory wrist, Parameters params) {}

  public static record Parameters(ArmState start, ArmState end) {}

  public static record CachedTrajectory(
      double[] initialPos,
      double[] finalPos,
      String[] constraintOverrides,
      double totalTime,
      double[] points) {}

  public static record StoredTrajectory(int id, List<CachedTrajectory> trajectories) {}

  private static final String cacheFilename = "arm_trajectory_cache.json";
  private static final String cacheRequestFilename = "arm_trajectory_cache_request.json";

  /** Reads cached trajectories and returns a list of them */
  public static Map<Integer, ArmTrajectory> loadTrajectories() {
    ObjectMapper mapper = new ObjectMapper();
    mapper.configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
    StoredTrajectory cache;
    try {
      cache =
          mapper.readValue(
              new File(Filesystem.getDeployDirectory(), cacheFilename), StoredTrajectory.class);
    } catch (IOException e) {
      DriverStation.reportError("COULD NOT LOAD TRAJECTORY CACHE, THE FILE CANNOT BE READ", false);
      return Map.of();
    }

    Map<Integer, ArmTrajectory> trajectories = new HashMap<Integer, ArmTrajectory>();

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
              ArmState.fromArray(cachedTrajectory.initialPos),
              ArmState.fromArray(cachedTrajectory.finalPos));

      trajectories.put(
          params.hashCode(),
          new ArmTrajectory(
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

    // Only use necessary presets for the current model of robot
    var presets =
        Constants.ROBOT_TYPE == RobotType.WHIPLASH_CLAW
            ? ArmState.OLD_PRESETS
            : ArmState.NEW_PRESETS;

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

    // run casadi solver on tmp file
    Process python = runPython();
    printOutput(python);
    System.exit(python.exitValue());
  }

  private static Process runPython() throws IOException, InterruptedException {
    ProcessBuilder pythonBuilder;
    String pythonPath;
    if (System.getProperty("os.name").startsWith("Windows")) {
      pythonPath = "python";
    } else {
      pythonPath = "./venv/bin/python";
    }

    pythonBuilder = new ProcessBuilder(pythonPath, "chronos" + File.separator + "run_gen_cache.py");
    System.out.println("Starting chronos process.");
    Process python = pythonBuilder.start();
    System.out.println("Generating cache... (this may take a while)");
    python.waitFor();
    return python;
  }

  private static void printOutput(Process process) throws IOException {
    BufferedReader stdoutReader = process.inputReader();
    BufferedReader stderrReader = process.errorReader();
    while (true) {
      String line = stdoutReader.readLine();
      if (line == null) {
        break;
      }
      System.out.println("Output: " + line);
    }
    while (true) {
      String line = stderrReader.readLine();
      if (line == null) {
        break;
      }
      System.out.println("Error: " + line);
    }
  }
}
