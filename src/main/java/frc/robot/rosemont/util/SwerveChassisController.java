package frc.robot.rosemont.util;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.rosemont.util.RoboMath.DualFactorSpeedController;
import frc.robot.subsystems.SwerveDrive;

public class SwerveChassisController {
    private double _xSpeed = 0;
    private double _ySpeed = 0;
    private double _rSpeed = 0;
    private double _speedModifier = 0;
    private double _heading = 0;

    private double _deadband = 0;

    private boolean _dualSpeedEnabled;

    private SlewRateLimiter _cardinalAccelerator;
    private SlewRateLimiter _angularAccelerator;

    private DualFactorSpeedController _dualSpeedCardinalController;
    private DualFactorSpeedController _dualSpeedAngularController;

    private ChassisSpeeds _swerveSpeeds;

    public SwerveChassisController(boolean dualSpeedEnabled, ChassisSpeeds swerveSpeeds) {
        this._dualSpeedEnabled = dualSpeedEnabled;

        this._cardinalAccelerator = new SlewRateLimiter(1);
        this._angularAccelerator = new SlewRateLimiter(1);

        this._dualSpeedCardinalController = new DualFactorSpeedController(1, 1);
        this._dualSpeedAngularController = new DualFactorSpeedController(1, 1);

        this._swerveSpeeds = swerveSpeeds;
    }

    public void configDualSpeedControllers(double[] cardinalSpeeds, double[] angularSpeeds) {
        if(_dualSpeedEnabled) {
            _dualSpeedCardinalController.configureSpeeds(cardinalSpeeds[0], cardinalSpeeds[1]);
            _dualSpeedAngularController.configureSpeeds(angularSpeeds[0], angularSpeeds[1]);
        } else {}
    }

    public void configAccelerators(double cardinalAccelLimit, double angularAccelLimiter) {
        _cardinalAccelerator = new SlewRateLimiter(cardinalAccelLimit);
        _angularAccelerator = new SlewRateLimiter(angularAccelLimiter);
    }

    public void configSafetyMeasures(double deadband) {
        _deadband = deadband;
    }

    public void setSwerveSpeeds(double xSpeed, double ySpeed, double rSpeed, double sModifier, double heading) {
        this._xSpeed = xSpeed;
        this._ySpeed = ySpeed;
        this._rSpeed = rSpeed;
        this._speedModifier = sModifier;
        this._heading = heading;
    }

    public ChassisSpeeds getSwerveSpeeds() {
        return null;
    }
}
