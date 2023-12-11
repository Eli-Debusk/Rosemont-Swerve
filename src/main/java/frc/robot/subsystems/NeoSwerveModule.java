package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.*;
import frc.robot.rosemont.util.REVNeo;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class NeoSwerveModule {

    ////Motor Controllers
    private final REVNeo driveNEO;
    private final REVNeo pivotNEO;

    ////Encoders
    private RelativeEncoder driveEncoder;
    private RelativeEncoder pivotEncoder;
    private CANCoder absoluteEncoder;

    //#CLASS INIT
    public NeoSwerveModule (
        int drive_neo_id,
        int pivot_neo_id,
        int absolute_encoder_id,
        boolean invert_drive_motor,
        boolean invert_pivot_motor
    ) {
        ////Motor Controllers
        driveNEO = new REVNeo(drive_neo_id);
        pivotNEO = new REVNeo(pivot_neo_id);

        ////Encoders
        driveEncoder = driveNEO.getEncoder();
        pivotEncoder = pivotNEO.getEncoder();
        absoluteEncoder = new CANCoder(absolute_encoder_id);

        ////Device Configuration
        //:i motor direction configuration
        driveNEO.setInverted(invert_drive_motor);
        pivotNEO.setInverted(invert_pivot_motor);

        //:i position conversion configuration
        driveEncoder.setPositionConversionFactor(NEOSwerveConstants.DriveRotationToMeter);
        pivotEncoder.setPositionConversionFactor(NEOSwerveConstants.PivotRotationToRadians);

        //:i velocity conversion configuration
        driveEncoder.setVelocityConversionFactor(NEOSwerveConstants.DriveRPMToMPS);
        pivotEncoder.setVelocityConversionFactor(NEOSwerveConstants.PivotRPMToRPS);

        //:i absolute sensor configuration
        absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

        //:i neo pid controller configuration
        pivotNEO.configPIDController(NEOSwerveConstants.kPivotProportional, 0, 0);
    }

    ////Retriving Sensor Positions
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return pivotEncoder.getPosition();
    }

    ////Retriving Sensor Velocities
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return pivotEncoder.getVelocity();
    }

    ////Retriving Absolute Sensor Position
    public double getAbsolutePosition() {
        return absoluteEncoder.getAbsolutePosition();
    }
    
    ////Resetting Relative Encoder Positions
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        pivotEncoder.setPosition(0);
    }

    ////Retrieving converted SwerveModuleState given velocity and angle
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    ////Setting the output of the module
    public void setDesiredState(SwerveModuleState state){
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        driveNEO.set(state.speedMetersPerSecond);
        pivotNEO.runToPosition(state.angle.getRadians());
    }

    ////Moving the module to the zero position of the Relative Encoder
    public void zeroModule() {
        driveNEO.set(0);
        pivotNEO.runToPosition(0);
    }

    ////Util Functions
    public void stop() {
        driveNEO.set(0);
        pivotNEO.set(0);
    }
}
