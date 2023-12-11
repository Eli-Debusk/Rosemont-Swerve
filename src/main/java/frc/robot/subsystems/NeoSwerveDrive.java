package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NEOSwerveConstants;

//#the main subsystem right here
public class NeoSwerveDrive extends SubsystemBase {

    ////Swerve Modules
    private final NeoSwerveModule leftFront = new NeoSwerveModule(
        NEOSwerveConstants.kLeftFrontDriveCAN_ID,
        NEOSwerveConstants.kLeftFrontPivotCAN_ID,
        NEOSwerveConstants.kLeftFrontAbsoluteEncoderCAN_ID,
        false,
        false
    );

    private final NeoSwerveModule leftBack = new NeoSwerveModule(
        NEOSwerveConstants.kLeftBackDriveCAN_ID,
        NEOSwerveConstants.kLeftBackPivotCAN_ID,
        NEOSwerveConstants.kLeftBackAbsoluteEncoderCAN_ID,
        false,
        false
    );

    private final NeoSwerveModule rightFront = new NeoSwerveModule(
        NEOSwerveConstants.kRightFrontDriveCAN_ID,
        NEOSwerveConstants.kRightFrontPivotCAN_ID,
        NEOSwerveConstants.kRightFrontAbsoluteEncoderCAN_ID,
        true,
        true
    );

    private final NeoSwerveModule rightBack = new NeoSwerveModule(
        NEOSwerveConstants.kRightBackDriveCAN_ID,
        NEOSwerveConstants.kRightBackPivotCAN_ID,
        NEOSwerveConstants.kRightBackAbsoluteEncoderCAN_ID,
        true,
        true
    );

    
}
