// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WestCoastDrive extends SubsystemBase {
  private CANSparkMax frontLeftSparkMax;
  private CANSparkMax backLeftSparkMax;
  private CANSparkMax frontRightSparkMax;
  private CANSparkMax backRightSparkMax;

 
  private RelativeEncoder rightRelativeEncoder;
  private RelativeEncoder leftRelativeEncoder;
  


  private MotorControllerGroup leftMotorGroup;
  private MotorControllerGroup rightMotorGroup;

  private DifferentialDrive driveBase;





  public WestCoastDrive() {
    //Left motors & group
    frontLeftSparkMax = new CANSparkMax(1, MotorType.kBrushless);
    backLeftSparkMax = new CANSparkMax(2, MotorType.kBrushless);
    leftMotorGroup = new MotorControllerGroup(backLeftSparkMax, frontLeftSparkMax);
    leftMotorGroup.setInverted(true);
    //Right motors & group
    frontRightSparkMax = new CANSparkMax(4, MotorType.kBrushless);
    backRightSparkMax = new CANSparkMax(3, MotorType.kBrushless);
    rightMotorGroup = new MotorControllerGroup(backRightSparkMax, frontRightSparkMax);


    driveBase = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

    rightRelativeEncoder = frontLeftSparkMax.getEncoder();
    leftRelativeEncoder = backLeftSparkMax.getEncoder();
  }


  //Main drive function - Controls are for tank drivestyle
  public void drive(double leftJoyStickY, double rightJoyStickY){
    driveBase.tankDrive(leftJoyStickY/2.2, rightJoyStickY/2);
    SmartDashboard.putNumber("LeftSpeed", leftRelativeEncoder.getVelocity());
    SmartDashboard.putNumber("RightSpeed", rightRelativeEncoder.getVelocity());
  

    

  }

  public void driveInStraightLine(){
    

  }

  


























  @Override
  public void periodic() {
      
  }
}
