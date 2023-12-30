package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.TeleOPConstants;
import frc.robot.commands.OperateShooterPower;
import frc.robot.commands.SwerveDriveController;
import frc.robot.subsystems.ExampleShooter;
import frc.robot.subsystems.SwerveDrive;

public class CommandCenter {
  
  public static final SwerveDrive swerveDrive = new SwerveDrive();

  public static final ExampleShooter shooter = new ExampleShooter();

  public final CommandXboxController controller = new CommandXboxController(TeleOPConstants.kController1Port);

  public CommandCenter() {

    swerveDrive.setDefaultCommand(new SwerveDriveController(
      swerveDrive, 
      controller
    ));

    configureBindings();
  }

  private void configureBindings() {
    controller.leftBumper().whileTrue(new OperateShooterPower(shooter, 1));
    controller.leftBumper().whileTrue(new OperateShooterPower(shooter, -1));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
