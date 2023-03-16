
package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmControlSubsystem extends SubsystemBase {
  
  public static enum ArmMode {
    NODE3,
    NODE2,
    NODE1,
    GNODE,
    NEUTRAL,
  }
      
  private final WPI_TalonFX leftPivotController = new WPI_TalonFX(ArmConstants.leftArmPivot);
  private final WPI_TalonFX rightPivotController = new WPI_TalonFX(ArmConstants.rightArmPivot);

  private final CANSparkMax _armExtensionMotor = new CANSparkMax(ArmConstants.telescopicArmSpark, MotorType.kBrushless);
  private final RelativeEncoder _armExtensionEncoder = _armExtensionMotor.getEncoder();  

  private final PIDController _pivotPIDController = new PIDController(0.02, 0, 0);
  private final PIDController _extensionPIDController = new PIDController(0.02, 0, 0);
  
  private ArmMode _armMode;
  private double _armAdjust;

  private double NODE1_ROT = 23;

  public ArmControlSubsystem() {       
    setConfig(false);
    SmartDashboard.putNumber("PivotkP", 2);
    SmartDashboard.putBoolean("armCoastMode", false);
  }

  public void setConfig(boolean isCoast){
    rightPivotController.configFactoryDefault();
    leftPivotController.configFactoryDefault();

    rightPivotController.follow(leftPivotController);

    rightPivotController.setInverted(TalonFXInvertType.OpposeMaster);
    leftPivotController.setInverted(ArmConstants.leftPivotInverted);

    rightPivotController.setNeutralMode(isCoast ? NeutralMode.Coast : NeutralMode.Brake);
    leftPivotController.setNeutralMode(isCoast ? NeutralMode.Coast : NeutralMode.Brake);

    rightPivotController.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    leftPivotController.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    leftPivotController.setSelectedSensorPosition(Units.radiansToRotations(ArmConstants.zeroAngleRad)*(1.0/ArmConstants.encoderResolution)*(1.0/ArmConstants.falconToFinalGear));
    rightPivotController.setSelectedSensorPosition(Units.radiansToRotations(ArmConstants.zeroAngleRad)*(1.0/ArmConstants.encoderResolution)*(1.0/ArmConstants.falconToFinalGear));

    _armExtensionMotor.setIdleMode(IdleMode.kBrake);
    _armExtensionMotor.setInverted(true);
    _armExtensionEncoder.setPositionConversionFactor(1);
    _armExtensionEncoder.setPosition(0);

    SmartDashboard.putNumber("InitialExtensionPosRaw", _armExtensionEncoder.getPosition());
    _armExtensionMotor.burnFlash();
    
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Adjust", _armAdjust);
    SmartDashboard.putNumber("Arm Pivot Encoder", 1);
    SmartDashboard.putNumber("Arm Extension Encoder", 1);

    if(_armMode == ArmMode.NEUTRAL){
      SmartDashboard.putString("Arm Mode", "NEUTRAL");
    }
    else if(_armMode == ArmMode.NODE1){
      SmartDashboard.putString("Arm Mode", "NODE1");
    }
    else if(_armMode == ArmMode.NODE2){
      SmartDashboard.putString("Arm Mode", "NODE2");
    }
    else if(_armMode == ArmMode.NODE3){
      SmartDashboard.putString("Arm Mode", "NODE3");
    }
    else if(_armMode == ArmMode.GNODE){
      SmartDashboard.putString("Arm Mode", "GNODE");
    }
  }

  //these functions assume the camera is on the front of the drivebase 
  public double getDistanceFromPivotToPose(double distanceFromCamera){
    return Math.sqrt(Math.pow(distanceFromCamera, 2) + Math.pow(ArmConstants.pivotPosInMetersY, 2));
  }
  /**
   * 
   * @param distanceFromCamera in meters
   * @return the desired angle in radians
   */


  public double getDesiredPivotAngle(double distanceFromCamera){
    return Math.atan(distanceFromCamera / ArmConstants.pivotPosInMetersY); //in radians
  }

  public boolean atAngleSetpoint(){
    return _pivotPIDController.atSetpoint();
  }


  public boolean atTelescopeSetpoint(){
    //return extensionPID.atSetpoint();
    return true;
  }

  public double getCurrentPivotRotation(boolean inRadians){
    //double rotation = pivotEncoder.getAbsolutePosition() - ArmConstants.pivotInitOffset;
    double rotation = (leftPivotController.getSelectedSensorPosition()) * ArmConstants.encoderResolution * ArmConstants.falconToFinalGear;

    rotation = Math.IEEEremainder(rotation, 1.0);
    
    if(inRadians){
      return rotation * 2 * Math.PI;
    }
    return rotation;
  }

  public double getCurrentExtensionIn(){
    return _armExtensionEncoder.getPosition();
  }
  
  public void armUp(){
    switch (_armMode) {
      case NEUTRAL:      
        _armMode = ArmMode.NODE1;
        break;
      case NODE1:
        _armMode = ArmMode.NODE2;
        break;
      case NODE2:
        _armMode = ArmMode.NODE3;
        break;
      case NODE3:
          break;
      case GNODE:
        break;
    }
  }

  public void armDown(){
  }
  
  public void holdArm(){
    if(_armMode == ArmMode.NODE1){
      // _extensionPIDController.setReference(NODE1_ROT + _armAdjust, kPosition);
    }
    else if(_armMode == ArmMode.NODE2){
      // _extensionPIDController.setReference(NODE2_ROT + _armAdjust, kPosition);
    }
  }
}
