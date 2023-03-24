// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CompConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.DriveMode;

public class AutoBalanceCommand extends CommandBase {
  
  private SwerveSubsystem swerveSubsystem;

  
  private boolean isOnChargingStation = false;

  private float yaw, pitch, roll;
  
  private PIDController velocityController;


  


  public AutoBalanceCommand(SwerveSubsystem swervee) {
    this.swerveSubsystem = swervee;

    velocityController = CompConstants.velocityController;
    velocityController.setTolerance(CompConstants.kAngleDeadZoneDeg);


    addRequirements(swervee);
  }


  @Override
  public void initialize() {

    isOnChargingStation = false;
    pitch=0; roll=0; yaw=0;

  }

  
  @Override
  public void execute() {


    this.pitch = this.swerveSubsystem.getPitch();
    this.roll = this.swerveSubsystem.getRoll();
    this.yaw = this.swerveSubsystem.getYaw();


    if (!this.isOnChargingStation){
      this.goForwardUntilOnChargeStation();
    }else if (Math.abs(this.roll) >= 2.5){
      this.doBalanceMethod1();
    }else{
      //brake
      this.swerveSubsystem.stopAllAndBrake();
    }
  }

  //only uses PID control
  private void doBalanceMethod1(){
    double pidOutput = velocityController.calculate(this.roll, 0);
    SmartDashboard.putNumber("BalancePIDOutput", pidOutput);


    this.swerveSubsystem.setMotors(pidOutput/14, 0, 0,DriveMode.AUTO,false);
  }
  

  private void goForwardUntilOnChargeStation(){
    if(Math.abs(this.roll) >= CompConstants.onChargeStationOrientation){
      this.isOnChargingStation = true;
      return;
    }

    this.swerveSubsystem.setMotors(CompConstants.entrySpeed, 0, 0,DriveMode.AUTO,false);

  }



  @Override
  public boolean isFinished() {
    return false; // <- add an isFINISHED!
  }
}