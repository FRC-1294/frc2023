package frc.robot.subsystems;

import frc.robot.InputDevice;

public class DriveInputSubsystem {
    public static enum InputType {
        XboxController,
        LogitechG3dJoystick
    }
    
    public static InputDevice selectInputDevice(InputType type) {
        if (type == InputType.XboxController) {
            return new XboxJoystick();
        } else {
            return new Joysticks();
        }
    }    
}

