package org.sciborgs1155.lib;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class TestableSubsystem extends SubsystemBase implements AutoCloseable {
    public abstract void close();
}
