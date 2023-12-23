package frc.rosemont.util;

public class RoboMath {
    public static double clip(double value, double min, double max) {
        return Math.max(min, Math.min(value, max));
    }

    public static double applyDeadband(double value, double deadband) {
        return Math.abs(value) > deadband ? value : 0.0;
    }

    public static class DualFactorSpeedController {
        private double _baseSpeed;
        private double _highSpeed;

        public DualFactorSpeedController(double baseSpeed, double highSpeed) {
            this._baseSpeed = baseSpeed;
            this._highSpeed = highSpeed;
        }

        public void configureSpeeds(double baseSpeed, double highSpeed) {
            this._baseSpeed = baseSpeed;
            this._highSpeed = highSpeed;
        }

        public double calculate(double factor) {
            double error = _highSpeed - _baseSpeed;

            return _baseSpeed + (error * factor);
        }
    }
}
