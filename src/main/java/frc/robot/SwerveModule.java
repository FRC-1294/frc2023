package frc.robot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveModule {
    
    public final boolean HasAbsoluteEncoder;
    private boolean absEncoderInverted;
    private AnalogEncoder absEncoder;
    private double absEncoderOffsetRadians;

    private final CANSparkMax transMotor;
    private final RelativeEncoder transEncoder;

    private final CANSparkMax rotMotor;
    private final RelativeEncoder rotEncoder;
    
    private final PIDController rotationPIDController;    
        
    private final String swerveModuleInfo;
    
    public SwerveModule(
        int transMotorID, 
        int rotMotorID, 
        int absEncoderID,
        boolean transInverted, 
        boolean rotInverted, 
        double absEncoderOffsetInit,
        boolean absEncoderInverted, 
        boolean hasAbsEncoder) {
        
        StringBuilder buf = new StringBuilder();        
        this.HasAbsoluteEncoder = hasAbsEncoder;
        buf.append("SwerveModule: HasAbsoluteEncoder = " + String.valueOf(hasAbsEncoder) + ", " );

        if (this.HasAbsoluteEncoder){
            absEncoder = new AnalogEncoder(absEncoderID);
            buf.append("absEncoderID = " + absEncoderID + ", ");

            this.absEncoderInverted = absEncoderInverted;
            buf.append("absEncoderInverted = " +  String.valueOf(absEncoderInverted) + ", ");
    
            this.absEncoderOffsetRadians = absEncoderOffsetInit;
            buf.append("absEncoderOffsetRadians = " +  absEncoderOffsetRadians + ", ");    
        }

        transMotor = new CANSparkMax(transMotorID, MotorType.kBrushless);
        buf.append("transMotorID = " + transMotorID + ", ");

        transMotor.setInverted(transInverted);
        buf.append("transInverted = " + transInverted + ", ");

        transEncoder = transMotor.getEncoder();

        rotMotor = new CANSparkMax(rotMotorID, MotorType.kBrushless);
        buf.append("rotMotorID = " + rotMotorID + ", ");

        rotMotor.setInverted(rotInverted);
        buf.append("rotInverted = " + rotInverted + ", ");

        rotEncoder = rotMotor.getEncoder();
                       
        rotationPIDController = new PIDController(
            Constants.kSwerveModuleSteeringMotorPIDConstants.kP,
            Constants.kSwerveModuleSteeringMotorPIDConstants.kI, 
            Constants.kSwerveModuleSteeringMotorPIDConstants.kD);

        buf.append("rotationPIDController: " + Constants.kSwerveModuleSteeringMotorPIDConstants);

        rotationPIDController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
        swerveModuleInfo = buf.toString();
    }

    public double getTransPosition(){
        return transEncoder.getPosition(); 
    }

    public double getRotPosition(){
        return rotEncoder.getPosition();
    }

    public double getTransVelocity(){
        return transEncoder.getVelocity();
    }

    public double getRotVelocity(){
        return rotEncoder.getVelocity();
    }

    public double getUniversalEncoderRad(){
        if (this.HasAbsoluteEncoder) {
            double angle = absEncoder.getAbsolutePosition();
            angle*=2.0 * Math.PI;
            angle-= absEncoderOffsetRadians;
            angle = this.absEncoderInverted ? -angle : angle;
            return angle;
        }
        return 0;
    }

    public void resetEncoders(){
        transEncoder.setPosition(0);
        rotEncoder.setPosition(0); 
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getTransVelocity(),new Rotation2d(getRotPosition()*2*Math.PI/18));
    }

    public void setDesiredState(SwerveModuleState desiredState){
        
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

        // if (this.m_transInverted) {
        //     transMotor.set(-desiredState.speedMetersPerSecond/Constants.maxSpeed);
        // }
        transMotor.set(desiredState.speedMetersPerSecond / Constants.kDriveMotorMaxSpeedMeterPerSecond);

        SmartDashboard.putNumber("RotationPosition" + toString(), getRotPosition());
        SmartDashboard.putNumber("DesiredRotation" + toString(), desiredState.angle.getRadians());
        rotMotor.set(rotationPIDController.calculate(rotEncoder.getPosition()*Constants.kAngularEncoderConversionFactor, desiredState.angle.getRadians()));                
    }

    public void updatePositions(Double setPoint){
        rotationPIDController.setPID(Global.kP, Global.kI, Global.kD);

        //TODO: Validate
        rotationPIDController.disableContinuousInput();

        double sp = rotationPIDController.calculate(rotEncoder.getPosition()*2*Math.PI/18, setPoint);
        rotMotor.set(sp);
    }

    public void returnToOrigin(){
        System.out.println("In PID loop");
        rotMotor.set(rotationPIDController.calculate(rotEncoder.getPosition()*2*Math.PI/18, 0));
        rotationPIDController.setTolerance(0);
    }

    public SwerveModulePosition getModulePos(){
        return new SwerveModulePosition(transEncoder.getPosition()*Constants.kDriveEncoderRPM2MeterPerSec,new Rotation2d(getRotPosition()));
    }

    public void stop() {
        transMotor.set(0);
        rotMotor.set(0);
    }

    public PIDController getPIDController(){
        return this.rotationPIDController;
    }

    public void setPidController(double p, double i, double d){
        rotationPIDController.setP(p);
        rotationPIDController.setI(i);
        rotationPIDController.setD(d);
    }

    @Override
    public String toString() {
        return swerveModuleInfo;
    }
}