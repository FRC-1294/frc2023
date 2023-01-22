package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.InputDevice;

public class XboxJoystick implements InputDevice {

    private final XboxController xController = new XboxController(Constants.kXboxControllerPort);
    
    @Override
    public boolean resetGyro() {
        return xController.getRawButton(0);
    }

    @Override
    public double getX() {
        return -xController.getRawAxis(Constants.kXboxControllerXAxis);
    }

    @Override
    public double getY() {
        return -xController.getRawAxis(Constants.kXboxControllerYAxis);
    }

    @Override
    public double getRot() {
        return xController.getRawAxis(Constants.kXboxControllerRotAxis);
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
        return xController.getLeftTriggerAxis() > 0.5;
    }    
}
