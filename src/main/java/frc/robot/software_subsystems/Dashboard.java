package frc.robot.software_subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveDrive;

public class Dashboard {

    ////VARIABLE INITIALIZATION

    //(i) Subsystem Classes
    private SwerveDrive swerveDrive;

    //(i) Odometry Class and Field2D
    private final SwerveDriveOdometry odometry;
    private final Field2d field;

    ////CLASS INITIALIZATION
    public Dashboard(SwerveDrive swerveDrive) {
        //(i) Defining Variables
        this.swerveDrive = swerveDrive;
        this.field = new Field2d();

        //(f) -> Constructs new odometry object with initializing fixed values
        this.odometry = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, new Rotation2d(0), null);
    }

    ////EVENT FUNCTIONS
    public void init() {
        //(f) -> Adding the field data to NetworkTables
        SmartDashboard.putData("Robot Field", field);
    }

    public void periodic() {
        //(f) -> Updates Odometry with chassis information and updates field using odometry information
        odometry.update(swerveDrive.getRotation2D(), swerveDrive.getSwerveModulePositions()); 
        field.setRobotPose(odometry.getPoseMeters()); 

        //(f) -> Updates Smartdashboard with Motor Safety information
        double[] tempuratures = swerveDrive.reportMotorTemps();

        SmartDashboard.putNumber("MTEMP : LF Drive", tempuratures[0]);
        SmartDashboard.putNumber("MTEMP : LF Pivot", tempuratures[1]);
        SmartDashboard.putNumber("MTEMP : LB Drive", tempuratures[2]);
        SmartDashboard.putNumber("MTEMP : LB Pivot", tempuratures[3]);
        SmartDashboard.putNumber("MTEMP : RF Drive", tempuratures[4]);
        SmartDashboard.putNumber("MTEMP : RF Pivot", tempuratures[5]);
        SmartDashboard.putNumber("MTEMP : RB Drive", tempuratures[6]);
        SmartDashboard.putNumber("MTEMP : RB Pivot", tempuratures[7]);
    }
}
