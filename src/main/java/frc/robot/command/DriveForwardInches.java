package frc.robot.command;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drivetrain;

public class DriveForwardInches extends CommandBase {
    private final Drivetrain _drivetrain;
    private final double _xSpeedMetersPerSecond;
    private final double _distanceInches;

    public DriveForwardInches(Drivetrain drivetrain, double xSpeedMetersPerSecond, double distanceInches) {
        _drivetrain = drivetrain;
        _xSpeedMetersPerSecond = xSpeedMetersPerSecond;
        _distanceInches = distanceInches;
        addRequirements(_drivetrain);
    }

    @Override
    public void initialize() {
        _drivetrain.stopModules();
        _drivetrain.resetEncoders();
    }

    @Override
    public void execute() {
        _drivetrain.drive(_xSpeedMetersPerSecond, 0, 0);
    }

    @Override
    public void end(boolean interrupted) {
        _drivetrain.stopModules();
    }

    @Override
    public boolean isFinished() {
        double inchesTraveled = Units.metersToInches(_drivetrain.getRobotPose().getX());
        return Math.abs(inchesTraveled) >= _distanceInches;
    }

}