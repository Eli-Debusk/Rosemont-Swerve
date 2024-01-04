package frc.rosemont.util.controllers;

public class DualFactorSpeedController {
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
