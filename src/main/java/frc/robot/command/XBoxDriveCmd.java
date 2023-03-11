package frc.robot.command;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Drivetrain;

public class XBoxDriveCmd extends CommandBase{
    private final Drivetrain _drivetrain;

    // Slew rate limiters to make joystick inputs more gentle; 1/3sec from 0 to 1 
    private final SlewRateLimiter _xSpeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter _ySpeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter _rotationLimiter = new SlewRateLimiter(3);


    public XBoxDriveCmd(Drivetrain drivetrain) {
        _drivetrain = drivetrain;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double xSpeed = MathUtil.applyDeadband(Constants.kXboxController.getLeftY(), 0.02);
        double ySpeed = MathUtil.applyDeadband(Constants.kXboxController.getLeftX(), 0.02);
        double thetaSpeed = MathUtil.applyDeadband(Constants.kXboxController.getRightX(), 0.02);

        xSpeed = -_xSpeedLimiter.calculate(xSpeed) * Constants.kDrivingMotorMaxSpeedMetersPerSecond;
        ySpeed = -_ySpeedLimiter.calculate(ySpeed) * Constants.kDrivingMotorMaxSpeedMetersPerSecond;
        thetaSpeed = -_rotationLimiter.calculate(thetaSpeed) * Constants.kSteeringMotorMaxSpeedRadiansPerSecond;

        _drivetrain.drive(xSpeed, ySpeed, thetaSpeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        _drivetrain.stopModules();
    }
}