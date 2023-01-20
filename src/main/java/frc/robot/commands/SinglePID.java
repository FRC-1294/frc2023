// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Global;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Joysticks;
import frc.robot.subsystems.SwerveSubsystem;

public class SinglePID extends CommandBase {
  SwerveSubsystem swerve;
  SwerveModule selectedModule;
  Joysticks joys;
  Double sp;
  public SinglePID(SwerveModule module) {
    System.out.println("in the commanfd :)");
    this.selectedModule = module;
    SmartDashboard.putNumber("kP", Global.kP);
    SmartDashboard.putNumber("kI", Global.kI);
    SmartDashboard.putNumber("kD", Global.kD);
    SmartDashboard.putNumber("setPointReal", Global.tuningSetpoint);    
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    System.out.println("in the execute");

    Global.kP = SmartDashboard.getNumber("kP", 0);
    Global.kI = SmartDashboard.getNumber("kI", 0);
    Global.kD = SmartDashboard.getNumber("kD", 0);

    SmartDashboard.putNumber("Setpoint", 0);
    
    SmartDashboard.putNumber("setPointReal", Global.tuningSetpoint);
    sp = SmartDashboard.getNumber("setPointReal", 0);  
    SmartDashboard.putNumber("Module1CurrentROT",this.selectedModule.getRotPosition());
    
    selectedModule.updatePositions(sp);
    
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
