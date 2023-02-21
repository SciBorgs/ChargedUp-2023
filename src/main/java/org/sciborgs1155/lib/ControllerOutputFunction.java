package org.sciborgs1155.lib;

import edu.wpi.first.math.MathUtil;

/** Operator utility function. Maps raw input [-1 to 1] to robot output [-1 to 1]. */
@FunctionalInterface
public interface ControllerOutputFunction {
    double map(double raw);

    /**
     * y = [ base^(abs(x)^power) ] / (a - 1)
     *
     * @param power within the range (1.0, infinity)
     * @param base within the range (1.0, infinity)
     * @return
     */
    static ControllerOutputFunction powerExp(double power, double base) {
        return raw ->
                Math.signum(raw) * (Math.pow(base, Math.pow(Math.abs(raw), power)) - 1) / (base - 1);
    }

    static ControllerOutputFunction power(double power) {
        return raw -> Math.signum(raw) * Math.pow(Math.abs(raw), power);
    }

    default ControllerOutputFunction clamped(double limit) {
        return raw -> MathUtil.clamp(ControllerOutputFunction.this.map(raw), -limit, limit);
    }

    default ControllerOutputFunction scaled(double factor) {
        return raw -> factor * ControllerOutputFunction.this.map(raw);
    }
}
