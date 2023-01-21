// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Global;
import frc.robot.InputDevice;
import frc.robot.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;

public class PIDtuning extends CommandBase {
  SwerveSubsystem swerveee;
  InputDevice inputDevice;

  public PIDtuning(InputDevice inputDevice,SwerveSubsystem swerve) {
    this.swerveee = swerve;
    this.inputDevice = inputDevice;
    addRequirements(swerveee);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(Global.kP != SmartDashboard.getNumber("p", 0) || Global.kI != SmartDashboard.getNumber("i", 0) ||  Global.kI != SmartDashboard.getNumber("i", 0)){
      swerveee.setAllPIDControllers(SmartDashboard.getNumber("p",0), SmartDashboard.getNumber("i", 0), SmartDashboard.getNumber("d", 0));
      Global.kP = SmartDashboard.getNumber("p", 0);
      Global.kI = SmartDashboard.getNumber("i", 0);
      Global.kD = SmartDashboard.getNumber("d", 0);
      if (this.inputDevice.getIncPID()){
        Global.tuningSetpoint+=0.1;
      }else if(this.inputDevice.getDecPID()){
        Global.tuningSetpoint-=0.1;
      }
      SmartDashboard.putNumber("setPointReal", Global.tuningSetpoint);
      for (SwerveModule mod: swerveee.getRawModules()){
        mod.updatePositions(Global.tuningSetpoint);
      }
      
      SmartDashboard.putNumber("Module1CurrentROT",swerveee.getRawModules()[0].getRotPositionRadians());
      SmartDashboard.putNumber("Module2CurrentROT", swerveee.getRawModules()[1].getRotPositionRadians());
      SmartDashboard.putNumber("Module3CurrentROT", swerveee.getRawModules()[2].getRotPositionRadians());
      SmartDashboard.putNumber("Module4CurrentROT", swerveee.getRawModules()[3].getRotPositionRadians());
  }}

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
