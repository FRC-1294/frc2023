package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;

public class Joysticks {
    Joystick rJoystick;
    Joystick tJoystick;
    public Joysticks(){
        rJoystick = new Joystick (Constants.kRotationJoystickPort);
        tJoystick = new Joystick (Constants.kDrivingJoystickPort);
    }
    public boolean resetGyro(){return rJoystick.getRawButton(3);}
    public double getX(){return tJoystick.getY();}
    public double getY(){return -tJoystick.getX();}
    public double getRot(){return -rJoystick.getX();}
    public boolean getIncPID(){return tJoystick.getRawButton(5);}
    public boolean getDecPID(){return rJoystick.getRawButton(4);}
    public boolean getRobotOriented(){return tJoystick.getTrigger();}
    
}