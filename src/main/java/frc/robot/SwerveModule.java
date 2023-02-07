package frc.robot;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveModule {

    private int m_MotorTransID;
    private int m_MotorRotID;
    private int m_UniversalEncoderID;
    public CANSparkMax transMotor;
    private CANSparkMax rotMotor;
    private RelativeEncoder transEncoder;
    private RelativeEncoder rotEncoder;
    public AnalogEncoder universalEncoder;
    public SparkMaxPIDController rotPID;
    public PIDController rotationPIDController;
    public PIDController transController = new PIDController(Constants.kP, Constants.kI, Constants.kD);
    private Boolean isAbsoluteEncoder;
    private Boolean m_transInverted;
    private Boolean m_rotInverted;   
    
    /**
     * Creates a new Swerve Module
     * @param motorTransID the CAN id of the translation motor
     * @param motorTransID the CAN id of the rotation motor
     * @param universalEncoderID the analog port number that the absolute (universal) encoder is connected to
     * @param transInverted is the translation motor inverted?
     * @param rotInverted is the rotation motor inverted?
     * @param universalEncoderOffsetinit the inital offset of the absolute (universal) encoder
     * @param universalEncoderInverted is the absolute (universal) encoder inverted?
     * @param pidController the PIDController for the rotation motor
     * @param transCrontroller the PIDController for the translation motor
     */
    public SwerveModule(int motorTransID, int motorRotID, int universalEncoderID,
     Boolean transInverted, Boolean rotInverted, double universalEncoderOffsetinit,
     Boolean universalEncoderInverted, PIDController pidController,PIDController transController){
        this.isAbsoluteEncoder=isAbsEncoder;
        this.m_MotorTransID = motorTransID;
        this.m_UniversalEncoderID = universalEncoderID;
        this.m_MotorRotID = motorRotID;
        this.m_transInverted = transInverted;
        this.m_rotInverted = rotInverted;

        transMotor = new CANSparkMax(this.m_MotorTransID, MotorType.kBrushless);
        
        rotMotor = new CANSparkMax(this.m_MotorRotID, MotorType.kBrushless);

        if (isAbsEncoder){
            universalEncoder = new AnalogEncoder(this.m_UniversalEncoderID);
            universalEncoder.setPositionOffset(universalEncoderOffsetinit);
            SmartDashboard.putNumber("Offset", universalEncoder.getPositionOffset());
        }


        transMotor.setInverted(this.m_transInverted);
        rotMotor.setInverted(this.m_rotInverted);


        transEncoder = transMotor.getEncoder();
        rotEncoder = rotMotor.getEncoder();
        rotationPIDController = pidController;
        rotationPIDController.enableContinuousInput(-Math.PI,Math.PI);       
        resetEncoders(); 

    }
    
    /**
     * @return Returns rotations of translation motor BEFORE GEAR RATIO
     */
    public double getTransPosition(){
        return transEncoder.getPosition(); 
    }

    /**
     * @return returns rotations of rotation motor BEFORE GEAR RATIO
     */
    public double getRotPosition(){
        return rotEncoder.getPosition();
    }

    /**
     * @return the RPM of Translation motor BEFORE GEAR RATIO
     */
    public double getTransVelocity(){
        return transEncoder.getVelocity(); 
    }
    
    /**
     * @return RPM of Rotation motor BEFORE GEAR RATIO
     */
    public double getRotVelocity(){
        return rotEncoder.getVelocity();
    }

    /**
     * resets the Absolute (Universal) Encoder Position
     */
    public void resetEncoders(){
        rotEncoder.setPosition((universalEncoder.getAbsolutePosition()-universalEncoder.getPositionOffset())*18);
    }

    /**
     * @return the SwerveModuleState
     */
    public SwerveModuleState getState(){
        return new SwerveModuleState(getTransVelocity()*Constants.RPMtoMPS*Constants.driveEncoderConversionFactortoRotations,
            new Rotation2d(getRotPosition()*Constants.angleEncoderConversionFactortoRad));
    }

    /**
     * Sets the desiredState of the SwerveModule
     * @param desiredState the desiredState of the SwerveModule
     */
    public void setDesiredState(SwerveModuleState desiredState){
        
        //Stops returning to original rotation

        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) 
        {
            stop();
            return;
        }

        //No turning motors over 90 degrees
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

        //PID Controller for both translation and rotation
        transMotor.set(transController.calculate(
            transEncoder.getVelocity()*Constants.driveEncoderConversionFactortoRotations*Constants.RPMtoMPS,
            desiredState.speedMetersPerSecond)/Constants.maxSpeedMPS);
        
        //transMotor.set(desiredState.speedMetersPerSecond/Constants.maxSpeed);
        //Keep this
        
        rotMotor.set(rotationPIDController.calculate(rotEncoder.getPosition()*Constants.angleEncoderConversionFactortoRad,
            desiredState.angle.getRadians()));
    }

    /**
     * Note: THIS METHOD WAS CREATED FOR PID TUNING ONLY
     * @param setPoint the setPoint
     */
    public void updatePositions(double setPoint){
        rotationPIDController.setPID(Constants.kP, Constants.kI, Constants.kD);
        rotationPIDController.disableContinuousInput();
        double sp = rotationPIDController.calculate((getRotPosition()-0.5)*2*Math.PI/18, setPoint);
        rotMotor.set(sp);
    }

    /**
     * Sets the rotation wheel to its original state
     * Used at start of match to make the wheel straight
     */
    public void returnToOrigin(){
        System.out.println("In PID loop");
        rotMotor.set(rotationPIDController.calculate(((getRotPosition()%18)*2*Math.PI/18), 0));
        rotationPIDController.setTolerance(0);
    }

    /**
     * Gets the position of the Swerve Module
     * @return the Position of the Swerve Module
     */
    public SwerveModulePosition getModulePos(){
        return new SwerveModulePosition(transEncoder.getPosition()/Constants.weirdAssOdVal*Constants.kDriveEncoderRot2Meter,
            new Rotation2d(getRotPosition()*Constants.angleEncoderConversionFactortoRad));
    }

    /**
     * Stops the translation motor and the rotation motor
     */
    public void stop() {
        transMotor.set(0);
        rotMotor.set(0);

    }

    /**
     * returns the PIDController of the rotation motor
     */
    public PIDController getPIDController(){
        return this.rotationPIDController;
    }
    // 0.003665908944166
    // 0.266807902319739
}