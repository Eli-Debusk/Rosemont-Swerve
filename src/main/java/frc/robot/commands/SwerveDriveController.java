package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveDrive;

public class SwerveDriveController extends CommandBase {
  
  private final SwerveDrive swerveDriveSystem;

  private final Supplier<Double> speedXSupplier;
  private final Supplier<Double> speedYSupplier;
  private final Supplier<Double> speedRSupplier;

  private final SlewRateLimiter omniAccelLimiter;
  private final SlewRateLimiter angularAccelLimiter;

  public SwerveDriveController(
    SwerveDrive subsystem,
    Supplier<Double> xSupplier,
    Supplier<Double> ySupplier,
    Supplier<Double> rSupplier
  ) {
    this.swerveDriveSystem = subsystem;

    this.speedXSupplier = xSupplier;
    this.speedYSupplier = ySupplier;
    this.speedRSupplier = rSupplier;

    this.omniAccelLimiter = new SlewRateLimiter(SwerveConstants.kMaxPhysicalAccelerationTeleOP);
    this.angularAccelLimiter = new SlewRateLimiter(SwerveConstants.kMaxAngularAccelerationTeleOP);

    addRequirements(swerveDriveSystem);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
