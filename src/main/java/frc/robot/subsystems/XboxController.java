package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import frc.robot.InputDevice;

public class XboxController implements InputDevice {

    private final Joystick driverJoytick = new Joystick(Constants.kXboxControllerPort);
    
    @Override
    public boolean resetGyro() {
        throw new RuntimeException("Not yet implemented");
    }

    @Override
    public double getX() {
        return driverJoytick.getRawAxis(Constants.kXboxControllerYAxis);
    }

    @Override
    public double getY() {
        return driverJoytick.getRawAxis(Constants.kXboxControllerXAxis);
    }

    @Override
    public double getRot() {
        return driverJoytick.getRawAxis(Constants.kXboxControllerRotAxis);
    }

    @Override
    public boolean getIncPID() {
        throw new RuntimeException("Not yet implemented");
    }

    @Override
    public boolean getDecPID() {
        throw new RuntimeException("Not yet implemented");
    }

    @Override
    public boolean getRobotOriented() {
        return driverJoytick.getRawButton(Constants.kXboxControllerRobotOrientedButtonIdx);
    }    
}
