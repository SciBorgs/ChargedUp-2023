package org.sciborgs1155.lib;

import java.util.ArrayList;

public final class FunctionRegistry implements Runnable {
  private static FunctionRegistry instance = null;

  private ArrayList<Runnable> runnables;

  private FunctionRegistry() {}

  public static FunctionRegistry getInstance() {
    if (instance == null) {
      instance = new FunctionRegistry();
    }
    return instance;
  }

  public void add(Runnable runnable) {
    runnables.add(runnable);
  }

  @Override
  public void run() {
    for (Runnable runnable : runnables) runnable.run();
  }
}
