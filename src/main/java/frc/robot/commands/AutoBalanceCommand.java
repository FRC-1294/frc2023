// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Joysticks;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoBalanceCommand extends CommandBase {
 
  //after starting command we continue forward until gyro detects roll of 15 degrees
  //then we start tracking the displacement in order to avoid overshooting.

  //reseting the gyro does not reset the axis of the robot
  //which means pitch and roll will always be relative

  float roll, pitch, yaw;
  Pose2d initialPose, currentPose;

  //arbritrary - fix later with dynamic speed control :)
  double firstSpeed = .25;

  Joysticks joyee;
  SwerveSubsystem swervee;

  AHRS navx;


  float deadzone = 2.5f;

  boolean isOnChargingStation = false;

  //as of right now this command is not toggleable
  public AutoBalanceCommand(Joysticks joys, SwerveSubsystem swervee) {
    this.joyee = joys;
    this.swervee = swervee;

    addRequirements(swervee);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    roll = swervee.getRoll();
    pitch = swervee.getPitch();
    yaw = swervee.getYaw();

  
  }

  
  @Override
  public void execute() {
    roll = swervee.getRoll();
    pitch = swervee.getPitch();
    yaw = swervee.getYaw();

    goForwardSlowly();


    putNumbersInDashboard();

  }

  void goForwardSlowly(){
    if(Math.abs(pitch) <= deadzone){
      //robot is stable and on the carpet ground not charge station
      swervee.setMotors(0, firstSpeed, 0);
    }
    else if(Math.abs(pitch) - 15 <= deadzone && !isOnChargingStation){
      isOnChargingStation = true;

    }


    if (isOnChargingStation){
      if(Math.abs(pitch) > 2.5){
        swervee.setMotors(0, firstSpeed, 0);
      }else{
        //we dont
        swervee.stopAllAndBrake();
      }
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
