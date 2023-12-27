package frc.rosemont.util;

import java.util.function.Supplier;

public class Joystick2d {
    
    private Supplier<Double> y;
    private Supplier<Double> x;
    private boolean reverseY = false;

    public Joystick2d() {}

    public Joystick2d(boolean reverseY) {
        this.reverseY = reverseY;
    }

    public void set(Supplier<Double>[] xyaxes) {
        this.x = xyaxes[0];
        this.y = xyaxes[1];
    }

    public double getMagnitude() {
        return Math.sqrt(
            Math.pow(x.get(), 2) + Math.pow(reverseY == true ? -y.get() : y.get(), 2)
        );
    }

    public double[] getAxes() {
        return new double[] {
            x.get(),
            reverseY == true ? -y.get() : y.get()
        };
    }
}