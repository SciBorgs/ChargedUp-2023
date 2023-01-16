package org.sciborgs1155.lib;

import java.util.HashMap;
import java.util.Map.Entry;
import java.util.Set;

public final class FunctionRegistry {
  private static FunctionRegistry instance = null;

  public static double DEFAULT_RATE = 0.01;

  private HashMap<Runnable, Double> runnables;

  private FunctionRegistry() {
    runnables = new HashMap<>();
  }

  public static FunctionRegistry getInstance() {
    if (instance == null) {
      instance = new FunctionRegistry();
    }
    return instance;
  }

  public void add(Runnable runnable, double rate) {
    runnables.put(runnable, rate);
  }

  public void add(Runnable runnable) {
    add(runnable, DEFAULT_RATE);
  }

  public Set<Entry<Runnable, Double>> getEntries() {
    return runnables.entrySet();
  }
}
