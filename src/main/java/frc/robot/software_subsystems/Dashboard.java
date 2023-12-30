package frc.robot.software_subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveDrive;

public class Dashboard {

    ////Subsystem Classes
    private SwerveDrive swerveDrive;

    ////Odometry Class and Field2D
    private final SwerveDriveOdometry odometry;
    private final Field2d field;

    ////CLASS INITIALIZATION
    public Dashboard(SwerveDrive swerveDrive) {
        //Initializing objects
        this.swerveDrive = swerveDrive;
        this.field = new Field2d();

        //Constructs new odometry object with initializing fixed values
        this.odometry = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, new Rotation2d(0), null);
    }

    ////EVENT FUNCTIONS
    public void init() {
        //Adding the field data to NetworkTables
        SmartDashboard.putData("Robot Field", field);
    }

    public void periodic() {
        //Updates Odometry with chassis information and updates field using odometry information
        odometry.update(swerveDrive.getRotation2D(), swerveDrive.getSwerveModulePositions()); 
        field.setRobotPose(odometry.getPoseMeters()); 
    }
}