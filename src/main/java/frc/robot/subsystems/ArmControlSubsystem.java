
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmControlSubsystem extends SubsystemBase {
  
  private final WPI_TalonFX leftPivotMotorController = new WPI_TalonFX(Constants.leftArmPivot);
  private final WPI_TalonFX rightPivotMotorController = new WPI_TalonFX(Constants.rightArmPivot);

  private final AnalogEncoder armPivotEncoder = new AnalogEncoder(Constants.armPivotEncoderPort);
  
  double[] setRotations = {90};
  double currentRotation = Constants.minAngle;
  double desiredRotation = currentRotation;

  double differenceInRotation = desiredRotation - currentRotation;
  PIDController pivotPID = new PIDController(0.1, 0, 0);
   
  final double setSpeed = .90;

  double pivotPIDOutput = 0;

  public ArmControlSubsystem(Joysticks joys) {
    
  }

  @Override
  public void periodic() {

    pivotPeriodic();

  }

  void pivotPeriodic(){
    //set currentRotation with encoders
    currentRotation = armPivotEncoder.getAbsolutePosition()*360%360;

    differenceInRotation = desiredRotation - currentRotation;

    //do correction
    // if (Math.abs(differenceInRotation) > 7.5){
    //   leftPivotMotorController.set(setSpeed);
    //   rightPivotMotorController.set(setSpeed);
    // }else if (Math.abs(differenceInRotation) > 2){
    //   leftPivotMotorController.set(setSpeed);
    // }

    pivotPIDOutput = pivotPID.calculate(differenceInRotation, 0);
    
    if(Math.abs(differenceInRotation) > 2){
      leftPivotMotorController.set(pivotPIDOutput);
      rightPivotMotorController.set(-pivotPIDOutput);
    }
  }

  public void setDesiredRotation(double _desiredRotation){
    desiredRotation = _desiredRotation;
  }
}
