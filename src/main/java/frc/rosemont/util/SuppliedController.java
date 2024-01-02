package frc.rosemont.util;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SuppliedController {
    private final Supplier<Double> leftYSupplier;
    private final Supplier<Double> leftXSupplier;

    private final Supplier<Double> rightYSupplier;
    private final Supplier<Double> rightXSupplier;

    private final Supplier<Double> leftTriggerSupplier;
    private final Supplier<Double> rightTriggerSupplier;

    public double ly;
    public double lx;

    public double ry;
    public double rx;

    public double lt;
    public double rt;

    public SuppliedController(XboxController controller, boolean reverseLeftY) {
        this.leftYSupplier = () -> reverseLeftY == true ? -controller.getLeftY() : controller.getLeftY();
        this.leftXSupplier = () -> controller.getLeftX();

        this.rightYSupplier = () -> controller.getRightY();
        this.rightXSupplier = () -> controller.getRightX();

        this.leftTriggerSupplier = () -> controller.getLeftTriggerAxis();
        this.rightTriggerSupplier = () -> controller.getRightTriggerAxis();
    }

    public SuppliedController(CommandXboxController controller, boolean reverseLeftY) {
        this.leftYSupplier = () -> reverseLeftY == true ? -controller.getLeftY() : controller.getLeftY();
        this.leftXSupplier = () -> controller.getLeftX();

        this.rightYSupplier = () -> controller.getRightY();
        this.rightXSupplier = () -> controller.getRightX();

        this.leftTriggerSupplier = () -> controller.getLeftTriggerAxis();
        this.rightTriggerSupplier = () -> controller.getRightTriggerAxis();
    }

    public void update() {
        this.ly = leftYSupplier.get();
        this.lx = leftXSupplier.get();

        this.ry = rightYSupplier.get();
        this.rx = rightXSupplier.get();

        this.lt = leftTriggerSupplier.get();
        this.rt = rightTriggerSupplier.get();
    }
}
