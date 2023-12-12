package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.*;

import frc.robot.Constants.SwerveConstants;
import frc.robot.rosemont.util.NEOBrushlessMotor;
import frc.robot.rosemont.util.RoboMath;

////#Swerve Module Class, (using NEO Motors, and CTRE-CANCoder Absolute Encoder)
public class SwerveModule {
    
    ////DEVICE INITIALIZATION
    private final NEOBrushlessMotor driveNEO;
    private final NEOBrushlessMotor pivotNEO;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder pivotEncoder;
    private final CANCoder absoluteEncoder;

    ////CLASS INITIALIZATION
    public SwerveModule(
        int driveID,
        int pivotID,
        int cancoderID,
        boolean driveReverse,
        boolean pivotReverse
    ) {

        ////DEVICE DECLARATION
        driveNEO = new NEOBrushlessMotor(driveID);
        pivotNEO = new NEOBrushlessMotor(pivotID);

        driveEncoder = driveNEO.getEncoder();
        pivotEncoder = pivotNEO.getEncoder();
        absoluteEncoder = new CANCoder(cancoderID);

        ////DEVICE CONFIGURATION
        driveNEO.setInverted(driveReverse);
        pivotNEO.setInverted(pivotReverse);

        driveNEO.setIdleMode(IdleMode.kBrake);

        driveEncoder.setPositionConversionFactor(SwerveConstants.DriveRotationToMeter);
        driveEncoder.setVelocityConversionFactor(SwerveConstants.DriveRPMToMPS);

        pivotEncoder.setPositionConversionFactor(SwerveConstants.PivotRotationToRadians);
        pivotEncoder.setVelocityConversionFactor(SwerveConstants.PivotRPMToRPS);

        pivotNEO.configPIDController(SwerveConstants.kPivotProportional, 0, 0);
        pivotNEO.configPIDControllerCI(-Math.PI, Math.PI);
    }  

    ////FEEDBACK FUNCTIONS
    //Drive Encoders
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    //Pivot Encoders
    public double getPivotPosition() {
        return pivotEncoder.getPosition();  
    }

    public double getPivotVelocity() {
        return pivotEncoder.getVelocity();  
    }

    //Absolute Encoder
    public double getAbsolutePosition() {
        return absoluteEncoder.getAbsolutePosition();
    }

    ////ONLY USE FOR TELEMETRY, NOT FOR CALCULATIONS
    public double[] reportEncoderData() {
        return new double[] {
            driveEncoder.getPosition(), //Index 0
            driveEncoder.getVelocity(), //Index 1
            pivotEncoder.getPosition(), //Index 2
            pivotEncoder.getVelocity(), //Index 3
            absoluteEncoder.getAbsolutePosition() //Index 4
        };
    }

    ////MOVEMENT FUNCTIONS
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getPivotPosition()));
    }

    public void setModuleState(SwerveModuleState moduleState) {
        if (Math.abs(moduleState.speedMetersPerSecond) < 0.001) {
            return;
        }

        //Runs an optimization algorithm to reduce travel distance of the pivot motor
        moduleState = SwerveModuleState.optimize(moduleState, getModuleState().angle); 

        ////Motor Outputs:
        /*
        Gets the wanted velocity of the drive motor and divides it by the maximum speed 
        of the chassis to get the desired power output of the motor.
        */
        driveNEO.set(RoboMath.clip(moduleState.speedMetersPerSecond / SwerveConstants.kMaxPhysicalSpeed, -1, 1));

        //Retrieves the desired angle of the pivot motor and runs the motor to the desired angle using the NEOBrushlessMotor PID Controller
        pivotNEO.runToPosition(moduleState.angle.getRadians());
    }

    public void zeroModules() {
        driveNEO.stopMotor();
        pivotNEO.runToPosition(0);
    }

    ////UTIL FUNCTIONS
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        pivotEncoder.setPosition(0);
    }

    public void stopMotors() {
        driveNEO.stopMotor();
        pivotNEO.stopMotor();
    }
}
