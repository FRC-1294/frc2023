// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public interface InputDevice {
    public boolean resetGyro();
    public double  getX();
    public double  getY();
    public double  getRot();
    public boolean getIncPID();
    public boolean getDecPID();
    public boolean getRobotOriented();
}
