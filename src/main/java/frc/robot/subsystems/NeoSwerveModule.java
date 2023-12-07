package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class NeoSwerveModule {

    ////Motor Controllers
    private final CANSparkMax driveNEO;
    private final CANSparkMax pivotNEO;

    ////Encoders
    private RelativeEncoder driveEncoder;
    private RelativeEncoder pivotEncoder;
    private CANCoder absoluteEncoder;

    ////PID Controller 
    private final PIDController pivotPIDController;

    //#CLASS INIT
    public NeoSwerveModule (
        int drive_neo_id,
        int pivot_neo_id,
        int absolute_encoder_id,
        boolean invert_drive_motor,
        boolean invert_pivot_motor
    ) {
        ////Motor Controllers
        driveNEO = new CANSparkMax(drive_neo_id, MotorType.kBrushless);
        pivotNEO = new CANSparkMax(pivot_neo_id, MotorType.kBrushless);

        ////Encoders
        driveEncoder = driveNEO.getEncoder();
        pivotEncoder = pivotNEO.getEncoder();
        absoluteEncoder = new CANCoder(absolute_encoder_id);

        ////PID Controller
        pivotPIDController = new PIDController(NEOSwerveConstants.kProportional, 0, 0);

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

        absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

        pivotPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }
    public double getTurningPosition() {
        return pivotEncoder.getPosition();
    }
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();

    }
    public double getTurningVelocity() {
        return pivotEncoder.getVelocity();
    }

    public double getAbsolutePosition() {
        return absoluteEncoder.getAbsolutePosition();
    }
    
   public void resetEncoders() {
        driveEncoder.setPosition(0);
        pivotEncoder.setPosition(0);
   }

   public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
   }

   public void setDesiredState(SwerveModuleState state){
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        driveNEO.set(state.speedMetersPerSecond);
        pivotNEO.set(pivotPIDController.calculate(getTurningPosition(), state.angle.getRadians()));
   }

   public void stop() {
        driveNEO.set(0);
        pivotNEO.set(0);
   }

   public void zeroModule() {
        driveNEO.set(0);
        pivotNEO.set(pivotPIDController.calculate(getAbsolutePosition(), 0));
   }
}
