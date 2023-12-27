package frc.rosemont.util;

public class Joystick2d {
    
    private double y;
    private double x;
    private boolean reverseY = false;

    public Joystick2d() {}

    public Joystick2d(boolean reverseY) {
        this.reverseY = reverseY;
    }
}
