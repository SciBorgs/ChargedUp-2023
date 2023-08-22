// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.lib.constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import java.util.function.Consumer;

/** Add your docs here. */
public class SparkUtils {
  public static CANSparkMax create(int port, MotorType type, Consumer<SparkConfig> configuration) {
    var config = new SparkConfig(port, type);
    configuration.accept(config);
    return config.build();
  }
}
