package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
  public final static class SwerveConstants {
    
    //#CAN IDS 

    ////Assigning constants for SparkMax CAN ID's (drive motors)
    public static final int kLeftFrontDriveCAN_ID = 0;
    public static final int kLeftBackDriveCAN_ID = 1;

    public static final int kRightFrontDriveCAN_ID = 2;
    public static final int kRightBackDriveCAN_ID = 3;

    ////Assigning constants for SparkMax CAN ID's (pivot motors)
    public static final int kLeftFrontPivotCAN_ID = 4;
    public static final int kLeftBackPivotCAN_ID = 5;

    public static final int kRightFrontPivotCAN_ID = 6;
    public static final int kRightBackPivotCAN_ID = 7;

    ////Assigning constants for CANCoder CAN ID's (absolute encoders)
    public static final int kLeftFrontAbsoluteEncoderCAN_ID = 8;
    public static final int kLeftBackAbsoluteEncoderCAN_ID = 9;

    public static final int kRightFrontAbsoluteEncoderCAN_ID = 10;
    public static final int kRightBackAbsoluteEncoderCAN_ID = 11;

    //#MECHANICAL CONSTANTS 

    ////Creating mechanical constants to be used in conversion calulations
    //:i wheel circumference
    public static final double kWheelCircumference = Units.inchesToMeters(1.5) * Math.PI;

    //:i gear ratios
    public static final double kDriveGearRatio = 1 / 8.14;
    public static final double kPivotGearRatio = 1 / 21.4285714;

    //#CONVERSION CONSTANTS

    ////Creating conversion constants for encoder data
    //:i position conversions
    public static final double DriveRotationToMeter = kDriveGearRatio * Math.PI * kWheelCircumference;
    public static final double PivotRotationToRadians = kPivotGearRatio * 2 * Math.PI;

    //:i velocity conversions
    public static final double DriveRPMToMPS = DriveRotationToMeter / 60;
    public static final double PivotRPMToRPS = PivotRotationToRadians / 60;

    //#MOVEMENT CONSTANTS

    ////Creating PID Constants for Wheel Pivoting
    public static final double kPivotProportional = 0.5;
  }
}
