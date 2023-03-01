package org.sciborgs1155.lib;

import java.util.IdentityHashMap;
import java.util.Map;
import java.util.function.BiConsumer;

public final class CustomPeriodRunnables {

  public static final double DEFAULT_RATE = 0.01;
  private static final Map<Runnable, Double> runnables = new IdentityHashMap<>();

  private CustomPeriodRunnables() {}

  public static void add(Runnable runnable, double rate) {
    runnables.put(runnable, rate);
  }

  public static void add(Runnable runnable) {
    add(runnable, DEFAULT_RATE);
  }

  public static void forEachRunnable(BiConsumer<? super Runnable, Double> consumer) {
    runnables.entrySet().forEach(entry -> consumer.accept(entry.getKey(), entry.getValue()));
  }
}
