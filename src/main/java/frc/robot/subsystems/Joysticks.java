package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;

/** Add your docs here. */
public class Joysticks {
    Joystick rJoystick;
    Joystick tJoystick;
    
    /**
     * Creates new Joysticks
     */
    public Joysticks(){
        rJoystick = new Joystick (Constants.rotJoystickPort);
        tJoystick = new Joystick (Constants.transJoystickPort);
    }

    /**
     * @return the state of the Gyro reset button
     */
    public boolean resetGyro(){return rJoystick.getRawButton(3);}

    /**
     * @return the value of the displacement in X (i think)
     */
    public double getX(){return tJoystick.getY();}

    /**
     * @return the value of the displacement in Y (i think)
     */
    public double getY(){return -tJoystick.getX();}

    /**
     * @return the amount that the robot should rotate
     */
    public double getRot(){return -rJoystick.getX();}

    /**
     * @return the state of the button
     */
    public boolean getIncPID(){return tJoystick.getRawButton(5);}

    /**
     * @return the state of the button
     */
    public boolean getDecPID(){return rJoystick.getRawButton(4);}

    /**
     * @return the state of the button
     */
    public boolean getRobotOriented(){return tJoystick.getTrigger();}

    /**
     * @return the state of the button
     */
    public boolean trajectoryRun(){return tJoystick.getRawButton(3);}

    /**
     * @return the state of the button
     */
    public boolean aprilAlign(){return rJoystick.getRawButton(11);}

    /**
     * @return the state of the button
     */
    public boolean moveTo() {return tJoystick.getRawButton(7);}
}