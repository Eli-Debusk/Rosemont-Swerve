package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveDrive extends SubsystemBase {

    ////SWERVE MODULES
    //Left
    private final SwerveModule leftFront = new SwerveModule(
        SwerveConstants.kLeftFrontDriveCAN_ID,
        SwerveConstants.kLeftFrontPivotCAN_ID,
        SwerveConstants.kLeftFrontAbsoluteEncoderCAN_ID,
        false,
        false
    );

    private final SwerveModule leftBack = new SwerveModule(
        SwerveConstants.kLeftBackDriveCAN_ID,
        SwerveConstants.kLeftBackPivotCAN_ID,
        SwerveConstants.kLeftBackAbsoluteEncoderCAN_ID,
        false,
        false
    );

    //Right
    private final SwerveModule rightFront = new SwerveModule(
        SwerveConstants.kRightFrontDriveCAN_ID,
        SwerveConstants.kRightFrontPivotCAN_ID,
        SwerveConstants.kRightFrontAbsoluteEncoderCAN_ID,
        true,
        false
    );

    private final SwerveModule rightBack = new SwerveModule(
        SwerveConstants.kRightBackDriveCAN_ID,
        SwerveConstants.kRightBackPivotCAN_ID,
        SwerveConstants.kRightBackAbsoluteEncoderCAN_ID,
        true,
        false
    );

    ////Odometry Class and Field2D
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, new Rotation2d(0), null);
    private final Field2d field = new Field2d();

    ////KuaiLabs NavX Gyroscope
    private final AHRS gyroscope = new AHRS(Port.kMXP);


    ////CLASS INITIALIZATION
    public SwerveDrive() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading(); //Reseting gyroscope angle
            } catch (Exception e) {}
        })
        .start();

        SmartDashboard.putData("Field", field);
    }

    @Override 
    public void periodic() {
        odometry.update(
            getRotation2D(), 
            new SwerveModulePosition[] {
                leftFront.getModulePosition(),
                rightFront.getModulePosition(),
                leftBack.getModulePosition(),
                rightBack.getModulePosition()                
            }
        );
        field.setRobotPose(odometry.getPoseMeters());
    }

    ////UTIL FUNCTIONS
    public void zeroHeading() {
        gyroscope.reset(); 
    }

    ////FEEDBACK FUNCTIONS
    //IEEE Remainder value from gyroscope angle and value
    public double getHeading() {
        return Math.IEEEremainder(gyroscope.getAngle(), 360);
    }

    //Retrieving robot Rotation2D value
    public Rotation2d getRotation2D() {
        return Rotation2d.fromDegrees(getHeading());
    }

    //Returning an Array of absolute encoder values
    public double[] getAbsoluteModulePositions() {
        return new double[] {
            leftFront.getAbsolutePosition(),
            leftBack.getAbsolutePosition(),
            rightFront.getAbsolutePosition(),
            rightBack.getAbsolutePosition()
        };
    }

    //Returning an Array of relative encoder values
    public double[] getRelativeModulePositions() {
        return new double[] {
            leftFront.getPivotPosition(),
            leftBack.getPivotPosition(),
            rightFront.getPivotPosition(),
            rightBack.getPivotPosition()
        };
    }

    ////MOVEMENT FUNCTIONS
    //Stopping motors
    public void stopModules() {
        leftFront.stopMotors();
        leftBack.stopMotors();
        rightFront.stopMotors();
        rightBack.stopMotors();
    }

    //Setting desired states of all modules in the Swerve Drive
    public void setDriveState(SwerveModuleState[] driveState) {
        SwerveDriveKinematics.desaturateWheelSpeeds(driveState, SwerveConstants.kMaxPhysicalSpeed); //Normailizing motor speeds
        //Swerve Module States are in the format of an array with front 
        //modules at [0,1] and back modules at [2,3], [left, right] respectively

        leftFront.setModuleState(driveState[0]);
        leftBack.setModuleState(driveState[2]);

        rightFront.setModuleState(driveState[1]);
        rightBack.setModuleState(driveState[3]);
    }

    //Zeroing swerve modules
    public void zeroModules() {
        leftFront.zeroRelativeEncoder();
        leftBack.zeroRelativeEncoder();

        rightFront.zeroRelativeEncoder();
        rightBack.zeroRelativeEncoder();
    }
}