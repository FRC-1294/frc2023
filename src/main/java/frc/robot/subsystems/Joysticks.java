package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import frc.robot.InputDevice;

public class Joysticks implements InputDevice {
    Joystick rJoystick;
    Joystick tJoystick;

    public Joysticks(){
        rJoystick = new Joystick (Constants.kRotationJoystickPort);
        tJoystick = new Joystick (Constants.kDrivingJoystickPort);
    }

    @Override
    public boolean resetGyro(){ return rJoystick.getRawButton(3); }

    @Override
    public double getX() { return tJoystick.getY(); }

    @Override
    public double getY() { return -tJoystick.getX(); }

    @Override
    public double getRot() { return -rJoystick.getX(); }
    
    @Override
    public boolean getIncPID() { return tJoystick.getRawButton(5); }

    @Override
    public boolean getDecPID() { return rJoystick.getRawButton(4); }

    @Override
    public boolean getRobotOriented() {return tJoystick.getTrigger(); }
}