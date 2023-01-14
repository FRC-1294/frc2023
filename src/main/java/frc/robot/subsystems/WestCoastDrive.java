// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WestCoastDrive extends SubsystemBase {
  private CANSparkMax frontLeftSparkMax;
  private CANSparkMax backLeftSparkMax;
  private CANSparkMax frontRightSparkMax;
  private CANSparkMax backRightSparkMax;

  private MotorControllerGroup leftMotorGroup;
  private MotorControllerGroup rightMotorGroup;

  private DifferentialDrive driveBase;





  public WestCoastDrive() {
    //Left motors & group
    frontLeftSparkMax = new CANSparkMax(1, MotorType.kBrushless);
    backLeftSparkMax = new CANSparkMax(2, MotorType.kBrushless);
    leftMotorGroup = new MotorControllerGroup(backLeftSparkMax, frontLeftSparkMax);
    //Right motors & group
    frontRightSparkMax = new CANSparkMax(4, MotorType.kBrushless);
    backRightSparkMax = new CANSparkMax(3, MotorType.kBrushless);
    rightMotorGroup = new MotorControllerGroup(backRightSparkMax, frontRightSparkMax);

    driveBase = new DifferentialDrive(leftMotorGroup, rightMotorGroup);
  }


  //Main drive function - Controls are for arcade drivestyle
  public void drive(double joystick1Y, double joystick2X){
    driveBase.arcadeDrive(joystick1Y, joystick2X);

  }

  


























  @Override
  public void periodic() {
      
  }
}
