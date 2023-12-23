package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TeleOPConstants;
import frc.robot.commands.SwerveDriveController;
import frc.robot.subsystems.SwerveDrive;

public class CommandCenter {
  
  public final SwerveDrive swerveDrive = new SwerveDrive();

  public final XboxController controller = new XboxController(TeleOPConstants.kController1Port);

  public CommandCenter() {

    swerveDrive.setDefaultCommand(new SwerveDriveController(
      swerveDrive, 
      () -> controller.getLeftX(), 
      () -> controller.getLeftY(), 
      () -> controller.getRightX(),
      () -> controller.getRightTriggerAxis()
    ));

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return null;
  }
}
