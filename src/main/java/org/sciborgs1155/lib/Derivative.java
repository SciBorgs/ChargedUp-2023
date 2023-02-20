package org.sciborgs1155.lib;

import edu.wpi.first.wpilibj.Timer;
import java.util.function.DoubleUnaryOperator;

/** A derivative with respect to system time. */
public class Derivative implements DoubleUnaryOperator {

    private double lastValue;
    private double lastTime;
    private double out;

    /** Creates a derivative with initial value 0. */
    public Derivative() {
        lastValue = 0;
        lastTime = Timer.getFPGATimestamp();
    }

    /**
     * Finds the slope between the current and previous inputted value.
     *
     * @param value x
     * @return dx/dt
     */
    public double calculate(double value) {
        out = (value - lastValue) / (Timer.getFPGATimestamp() - lastTime);
        lastValue = value;
        lastTime = Timer.getFPGATimestamp();
        return out;
    }

    /** Returns the last calculated value. */
    public double getLastOutput() {
        return out;
    }

    @Override
    public double applyAsDouble(double value) {
        return calculate(value);
    }
}
