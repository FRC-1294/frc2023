// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Joysticks;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoBalanceCommand extends CommandBase {
 
  //after starting command we continue forward until gyro detects roll of 15 degrees
  //then we start tracking the displacement in order to avoid overshooting.

  float displacement, roll, pitch, yaw;

  //arbritrary - fix later with dynamic speed control :)
  double setSpeed = 1;

  

  public AutoBalanceCommand() {}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    displacement = 0;
    roll = SwerveSubsystem.navx.getRoll();
    pitch = SwerveSubsystem.navx.getPitch();
    yaw = SwerveSubsystem.navx.getYaw();
  }

  
  @Override
  public void execute() {
    roll = SwerveSubsystem.navx.getRoll();
    pitch = SwerveSubsystem.navx.getPitch();
    yaw = SwerveSubsystem.navx.getYaw();


    putNumbersInDashboard();

    if (Joysticks.getResetGyro()) {
      System.out.println("RESET GYRO!");
      SwerveSubsystem.navx.reset();
    }

  }

  void putNumbersInDashboard(){
    SmartDashboard.putNumber("Roll", roll);
    SmartDashboard.putNumber("Pitch", pitch);
    SmartDashboard.putNumber("Yaw", yaw);
  }

  
  @Override
  public void end(boolean interrupted) {
    
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
