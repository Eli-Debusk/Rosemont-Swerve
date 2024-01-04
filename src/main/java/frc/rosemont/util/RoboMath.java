package frc.rosemont.util;

public class RoboMath {
    public static double clip(double value, double min, double max) {
        return Math.max(min, Math.min(value, max));
    }

    public static double applyDeadband(double value, double deadband) {
        return Math.abs(value) > deadband ? value : 0.0;
    }

    public static double headingRemainder(double angle) {
        return Math.IEEEremainder(angle, 360);
    }
}
