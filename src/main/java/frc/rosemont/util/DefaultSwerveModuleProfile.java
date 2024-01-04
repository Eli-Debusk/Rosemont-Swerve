package frc.rosemont.util;

import frc.rosemont.util.RosemontConstants.*;

public class DefaultSwerveModuleProfile {  
     
    public final int driveCID;
    public final int pivotCID;
    public final int absEncoderCID;
    public final boolean driveReversed;
    public final boolean pivotReversed;

    public DefaultSwerveModuleProfile(int modulePosition) {

        if (modulePosition == SwerveModulePositions.LEFTBACK) {
            driveCID = 1;
            pivotCID = 5;
            absEncoderCID = 9;
            driveReversed = false;
            pivotReversed = false;

        } else if (modulePosition == SwerveModulePositions.LEFTFRONT) {
            driveCID = 0;
            pivotCID = 4;
            absEncoderCID = 8;
            driveReversed = false;
            pivotReversed = false;

        } else if (modulePosition == SwerveModulePositions.RIGHTBACK) {
            driveCID = 3;
            pivotCID = 7;
            absEncoderCID = 11;
            driveReversed = true;
            pivotReversed = false;

        } else if(modulePosition == SwerveModulePositions.RIGHTFRONT) {
            driveCID = 2;
            pivotCID = 6;
            absEncoderCID = 10;
            driveReversed = true;
            pivotReversed = false;
            
        } else {
            driveCID = 0;
            pivotCID = 0;
            absEncoderCID = 0;
            driveReversed = false;
            pivotReversed = false;
        }

    }
}
