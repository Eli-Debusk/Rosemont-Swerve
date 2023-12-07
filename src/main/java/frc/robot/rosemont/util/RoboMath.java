package frc.robot.rosemont.util;

public class RoboMath {
    public static double clip(double value, double min, double max) {
        return Math.max(min, Math.min(value, max));
    }
}
