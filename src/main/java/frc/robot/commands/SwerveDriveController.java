package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TeleOPConstants;
import frc.robot.subsystems.SwerveDrive;
import frc.rosemont.util.RoboMath;
import frc.rosemont.util.RoboMath.DualFactorSpeedController;

public class SwerveDriveController extends CommandBase {
  
  private final SwerveDrive swerveDriveSystem;

  private final Supplier<Double> speedXSupplier;
  private final Supplier<Double> speedYSupplier;
  private final Supplier<Double> speedRSupplier;
  private final Supplier<Double> speedModifier;
  private final Supplier<Boolean> resetPivotSupplier;

  private final SlewRateLimiter cardinalAccelLimiter;
  private final SlewRateLimiter angularAccelLimiter;

  private final DualFactorSpeedController cardinalDualLevelMaxSpeed;
  private final DualFactorSpeedController angularDualLevelMaxSpeed;

  public SwerveDriveController(
    SwerveDrive subsystem,
    CommandXboxController controller
  ) {
    this.swerveDriveSystem = subsystem;

    this.speedXSupplier = () -> controller.getLeftX();
    this.speedYSupplier = () -> -controller.getLeftY();
    this.speedRSupplier = () -> controller.getRightX();
    this.speedModifier = () -> controller.getRightTriggerAxis();
    this.resetPivotSupplier = () -> controller.x().getAsBoolean();

    this.cardinalAccelLimiter = new SlewRateLimiter(SwerveConstants.kMaxPhysicalAccelerationTeleOP);
    this.angularAccelLimiter = new SlewRateLimiter(SwerveConstants.kMaxAngularAccelerationTeleOP);

    this.cardinalDualLevelMaxSpeed = new DualFactorSpeedController(
      SwerveConstants.kNormalPhysicalSpeedTeleOP, 
      SwerveConstants.kFastPhysicalSpeedTeleOP
    );

    this.angularDualLevelMaxSpeed = new DualFactorSpeedController(
      SwerveConstants.kNormalAngularSpeedTeleOP, 
      SwerveConstants.kFastAngularSpeedTeleOP
    );

    addRequirements(swerveDriveSystem);
  }

  @Override
  public void initialize() {
    //swerveDriveSystem.zeroModulePivotPositions();
  }

  @Override
  public void execute() {
    double xSpeed = speedXSupplier.get();
    double ySpeed = speedYSupplier.get();
    double rSpeed = speedRSupplier.get();

    xSpeed = RoboMath.applyDeadband(xSpeed, TeleOPConstants.kSpeedDeadband);
    ySpeed = RoboMath.applyDeadband(ySpeed, TeleOPConstants.kSpeedDeadband);
    rSpeed = RoboMath.applyDeadband(rSpeed, TeleOPConstants.kSpeedDeadband);

    xSpeed = cardinalAccelLimiter.calculate(xSpeed) * cardinalDualLevelMaxSpeed.calculate(speedModifier.get());
    ySpeed = cardinalAccelLimiter.calculate(ySpeed) * cardinalDualLevelMaxSpeed.calculate(speedModifier.get());
    rSpeed = angularAccelLimiter.calculate(rSpeed) * angularDualLevelMaxSpeed.calculate(speedModifier.get());

    ChassisSpeeds chassisSpeeds;

    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      xSpeed, ySpeed, rSpeed, swerveDriveSystem.getRotation2D()
    );

    SwerveModuleState[] swerveStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(chassisSpeeds);

    swerveDriveSystem.setDriveState(swerveStates);

    if(resetPivotSupplier.get()) {
      swerveDriveSystem.zeroModulePivotPositions();
    } else {}
  }

  @Override
  public void end(boolean interrupted) {
    swerveDriveSystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}