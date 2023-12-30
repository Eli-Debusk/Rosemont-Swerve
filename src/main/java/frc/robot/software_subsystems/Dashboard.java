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
        this.swerveDrive = swerveDrive;

        this.odometry = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, new Rotation2d(0), null);
        field = new Field2d();
    }

    ////EVENT FUNCTIONS
    public void init() {
        SmartDashboard.putData("Robot Field", field);
    }

    public void periodic() {
        odometry.update(
            swerveDrive.getRotation2D(), 
            swerveDrive.getSwerveModulePositions()
        ); 
        field.setRobotPose(odometry.getPoseMeters()); 
    }
}
