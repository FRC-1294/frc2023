// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Input;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.DriveMode;

public class DefaultDriveCmd extends CommandBase {
  /** Creates a new DefaultDriveCmd. */
  SwerveSubsystem swerveee;
  private SlewRateLimiter xLimiter;
  private SlewRateLimiter yLimiter;
  private SlewRateLimiter turningLimiter;

  private boolean isPrecision = false;

  public DefaultDriveCmd(SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveee = swerve;

    xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccMPS);
    yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccMPS);
    turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccRadPS);
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(Input.getPrecisionsToggle()){
      isPrecision = !isPrecision;
    }


    double x = Input.getJoystickX();
    double y = Input.getJoystickY();
    double rot = Input.getRot();
    if (isPrecision){
      rot = rot/2;
      x = x/2;
      y = y/2;
      x = Math.abs(x) > 0.01 ? x : 0.0;
      y = Math.abs(y) > 0.01 ? y : 0.0;
      rot = Math.abs(rot) > 0.01 ? rot : 0.0;
    }
    else{
      x = Math.abs(x) > 0.1 ? x : 0.0;
      y = Math.abs(y) > 0.1 ? y : 0.0;
      rot = Math.abs(rot) > 0.05 ? rot : 0.0;
    }

      
    // 3. Make the driving smoother
    x = xLimiter.calculate(x) * DriveConstants.kPhysicalMaxSpeedMPS;
    y = yLimiter.calculate(y) * DriveConstants.kPhysicalMaxSpeedMPS;
    rot= turningLimiter.calculate(rot)
          * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond*3;
    swerveee.setMotors(x, y, rot, DriveMode.TELEOP);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //System.out.println("End");
    //swerveee.goToOrigin();
    //while (!getClose()){
      //swerveee.goToOrigin();
    //}   
  }

  public boolean getClose(){
    boolean gf = true;
    SwerveModule [] rawMods = swerveee.getRawModules();
    for(SwerveModule mod  : rawMods){
      if (!mod.getPIDController().atSetpoint()){
        gf = false;
      }
    }
    return gf;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
