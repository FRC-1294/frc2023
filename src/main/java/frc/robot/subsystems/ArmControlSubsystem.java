
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.ml.ANN_MLP;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

  private final CANSparkMax telescopicSpark = new CANSparkMax(Constants.telescopicArmSpark, MotorType.kBrushless);

  private final AnalogEncoder pivotEncoder = new AnalogEncoder(Constants.armPivotEncoderPort);
  private final RelativeEncoder telescopicEncoder = telescopicSpark.getEncoder();
  
  double[] setRotations = {90};
  double currentPivotRotation = Constants.minAngle;
  double desiredPivotRotation = currentPivotRotation;
  

  double differenceInRotation = desiredPivotRotation - currentPivotRotation;
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
    currentPivotRotation = pivotEncoder.getAbsolutePosition()*360%360;

    differenceInRotation = desiredPivotRotation - currentPivotRotation;

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

  public void setDesiredPivotRotation(double _desiredRotation){
    desiredPivotRotation = _desiredRotation;
  }

  public void setDesiredExtension(double _extension){

  }

  //these functions assume the camera is on the front of the drivebase 
  double getDistanceFromPivotToPose(double distanceFromCamera){
    return Math.sqrt(Math.pow(distanceFromCamera, 2) + Math.pow(Constants.pivotPosInMetersY, 2));
  }

  double getDesiredPivotAngle(double distanceFromCamera){
    return Math.atan(distanceFromCamera / Constants.pivotPosInMetersY);
  }

  
}
