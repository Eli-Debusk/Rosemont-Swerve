package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TeleOPConstants;
import frc.robot.rosemont.util.SwerveChassisController;
import frc.robot.subsystems.SwerveDrive;

public class SwerveDriveControllerExp extends CommandBase {
  
  private final SwerveDrive swerveDriveSystem;

  private final Supplier<Double> speedXSupplier;
  private final Supplier<Double> speedYSupplier;
  private final Supplier<Double> speedRSupplier;
  private final Supplier<Double> speedModifier;

  private final SwerveChassisController chassisController;

  public SwerveDriveControllerExp(
    SwerveDrive subsystem,
    Supplier<Double> xSupplier,
    Supplier<Double> ySupplier,
    Supplier<Double> rSupplier,
    Supplier<Double> speedModifierSupplier
  ) {
    this.swerveDriveSystem = subsystem;

    this.speedXSupplier = xSupplier;
    this.speedYSupplier = ySupplier;
    this.speedRSupplier = rSupplier;
    this.speedModifier = speedModifierSupplier;

    this.chassisController = new SwerveChassisController(true);

    this.chassisController.configAccelerators(
      SwerveConstants.kMaxPhysicalAccelerationTeleOP, 
      SwerveConstants.kMaxAngularAccelerationTeleOP
    );
    
    this.chassisController.configDualSpeedControllers(
      new double[]{SwerveConstants.kNormalPhysicalSpeedTeleOP, SwerveConstants.kNormalAngularSpeedTeleOP},
      new double[]{SwerveConstants.kNormalAngularSpeedTeleOP, SwerveConstants.kFastAngularSpeedTeleOP}
    );

    this.chassisController.configSafetyMeasures(TeleOPConstants.kSpeedDeadband);

    addRequirements(swerveDriveSystem);
  }

  @Override
  public void initialize() {
    swerveDriveSystem.zeroModules();
  }

  @Override
  public void execute() {
    chassisController.setSwerveSpeeds(
      speedXSupplier.get(), 
      speedYSupplier.get(), 
      speedRSupplier.get(), 
      speedModifier.get(), 
      swerveDriveSystem.getRotation2D()
    );

    SwerveModuleState[] swerveStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(chassisController.getSwerveSpeeds());

    swerveDriveSystem.setDriveState(swerveStates);
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
