package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.*;
import frc.rosemont.util.NEOBrushlessMotor;
import frc.rosemont.util.RoboMath;

////#Swerve Module Class, (using NEO Motors, and CTRE-CANCoder Absolute Encoder)
public class SwerveModule {
    
    ////DEVICE INITIALIZATION
    private final NEOBrushlessMotor driveNEO, pivotNEO;

    private final RelativeEncoder driveEncoder, pivotEncoder;
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
        driveNEO.setInverted(driveReverse); //Config direction of the drive motor
        pivotNEO.setInverted(pivotReverse); //Config direction of the pivot motor

        driveNEO.setIdleMode(IdleMode.kBrake); //Config the brake|coast mode of the drive motor

        ////Configuring conversion factors for encoder readings
        driveEncoder.setPositionConversionFactor(SwerveConstants.DriveRotationToMeter);
        driveEncoder.setVelocityConversionFactor(SwerveConstants.DriveRPMToMPS);

        pivotEncoder.setPositionConversionFactor(SwerveConstants.PivotRotationToRadians);
        pivotEncoder.setVelocityConversionFactor(SwerveConstants.PivotRPMToRPS);

        //Configuring PID Controller for the pivot motor
        pivotNEO.configPIDController(SwerveConstants.kPivotProportional, 0, 0);
        pivotNEO.configPIDControllerCI(-Math.PI, Math.PI);
    }  

    ////FEEDBACK FUNCTIONS
    //Drive Encoders
    public double getDrivePosition() { //Returns the current position of the drive encoder
        return driveEncoder.getPosition();
    }

    public double getDriveVelocity() { //Returns the current velocity of the drive encoder
        return driveEncoder.getVelocity();
    }

    //Pivot Encoders
    public double getPivotPosition() { //Returns the current position of the pivot encoder
        return pivotEncoder.getPosition();  
    }

    public double getPivotVelocity() { //Returns the current velocity of the pivot encoder
        return pivotEncoder.getVelocity();  
    }

    //Absolute Encoder
    public double getAbsolutePosition() { //Returns the current position of the absolute encoder
        return absoluteEncoder.getAbsolutePosition();
    }

    //Returning SwerveModulePosition
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getPivotPosition()));
    }

    ////ONLY USE FOR TELEMETRY, NOT FOR CALCULATIONS
    public double[] reportEncoderData() { //Returns an array of encoder data that can be used for telemetry
        return new double[] {
            driveEncoder.getPosition(), //Index 0
            driveEncoder.getVelocity(), //Index 1
            pivotEncoder.getPosition(), //Index 2
            pivotEncoder.getVelocity(), //Index 3
            absoluteEncoder.getAbsolutePosition() //Index 4
        };
    }

    ////MOVEMENT FUNCTIONS
    public SwerveModuleState getModuleState() { //Retrieves the current SwerveModuleState given the current motor information
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getPivotPosition()));
    }

    public void setModuleState(SwerveModuleState moduleState) { //Used to set the desired state of the module

        if (Math.abs(moduleState.speedMetersPerSecond) < 0.001) { //Discards miniscule speed values
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

    public void zeroModule() { //Uses the relative encoder to zero the wheel angle 
        driveNEO.stopMotor();
        pivotNEO.runToPosition(0);
    }

    ////UTIL FUNCTIONS
    public void resetEncoders() { //Resets the relative encoder's position to 0
        driveEncoder.setPosition(0);
        pivotEncoder.setPosition(0);
    }

    public void stopMotors() { //Stops the drive and pivot motors
        driveNEO.stopMotor();
        pivotNEO.stopMotor();
    }

    public void zeroPivotEncoderToAbs() {
        pivotEncoder.setPosition(getAbsolutePosition());
    }
}