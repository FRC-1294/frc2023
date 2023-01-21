/*----------------------------------------------------------------------------*/
/* Copyright (c) 2023 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI.Port;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.InputDevice;
import frc.robot.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {
  
  // Bevel Gear must be facing to the left in order to work
  private final SwerveModule frontLeft = new SwerveModule(
    Constants.kFrontLeftDriveMotorSparkID, 
    Constants.kFrontLeftSteerMotorSparkID, 
    0,
    false, 
    true,
    0,
    false, 
    false);

  private final SwerveModule frontRight = new SwerveModule(
    Constants.kFrontRightDriveMotorSparkID, 
    Constants.kFrontRightSteerMotorSparkID,
    0,
    true,
    true,
    0,
    false, 
    false);

  private final SwerveModule backLeft = new SwerveModule(
    Constants.kBackLeftDriveMotorSparkID, 
    Constants.kBackLeftSteerMotorSparkID,
    0,
    false,
    true,
    0,
    false, 
    false);

  private final SwerveModule backRight = new SwerveModule(
    Constants.kBackRightDriveMotorSparkID, 
    Constants.kBackRightSteerMotorSparkID,
    0,
    true,
    true,
    0,
    false, 
    false);

  private final SwerveDriveOdometry odometer;

  private final AHRS navx = new AHRS(Port.kMXP);
  
  private final InputDevice inputDevice;

  public SwerveSubsystem(InputDevice inputDevice) {

    this.odometer = new SwerveDriveOdometry(
      Constants.kSwerveDriveKinematics, 
      getRotation2d(), 
      this.getModuleStates());

    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();

    SmartDashboard.putNumber("p", 0);
    SmartDashboard.putNumber("i", 0);
    SmartDashboard.putNumber("d", 0);

    this.inputDevice = inputDevice;    
    resetGyro();    
    resetRobotPose();
  }

  @Override
  public void periodic() {  
    odometer.update(getRotation2d(), getModuleStates());
    if(this.inputDevice.resetGyro()){resetGyro();}
  }
  public void resetGyro(){
    navx.reset();
  }

  public double getHeading(){
    return Math.IEEEremainder(navx.getAngle(), 360);
  }

  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kTeleDriveMaxSpeedMetersPerSecond*10);
    SmartDashboard.putNumber("Module1ROT", desiredStates[0].angle.getRadians());
    SmartDashboard.putNumber("Module2ROT", desiredStates[1].angle.getRadians());
    SmartDashboard.putNumber("Module3ROT", desiredStates[2].angle.getRadians());
    SmartDashboard.putNumber("Module4ROT", desiredStates[3].angle.getRadians());
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  public SwerveModulePosition[] getModuleStates(){
    return(new SwerveModulePosition[] {
      frontLeft.getModulePos(),
      frontRight.getModulePos(),
      backLeft.getModulePos(),
      backRight.getModulePos()});
  }
  
  public void setMotors(double x, double y, double rot) {

    ChassisSpeeds chassisSpeeds = this.inputDevice.getRobotOriented() ? 
      new ChassisSpeeds(x, y, rot) : 
      ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, getRotation2d());

    SwerveModuleState[] moduleStates = Constants.kSwerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    this.setModuleStates(moduleStates);

    SmartDashboard.putNumber("Module1CurrentROT", frontLeft.getRotPositionRadians());
    SmartDashboard.putNumber("Module2CurrentROT", frontRight.getRotPositionRadians());
    SmartDashboard.putNumber("Module3CurrentROT", backLeft.getRotPositionRadians());
    SmartDashboard.putNumber("Module4CurrentROT", backRight.getRotPositionRadians());
    SmartDashboard.putNumber("ChassisSpeeds POT", chassisSpeeds.omegaRadiansPerSecond);
    SmartDashboard.putNumber("ChassisSpeed X", chassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("ChassisSpeed Y", chassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Heading", getHeading());
  }

  public void resetRobotPose(){
    odometer.resetPosition(getRotation2d(), getModuleStates(), getRobotPose());
  }

  public Pose2d getRobotPose(){
    return odometer.getPoseMeters();
  }

  public void goToOrigin(){
    System.out.println("executed");
    frontRight.returnToOrigin();
    frontLeft.returnToOrigin();
    backLeft.returnToOrigin();
    backRight.returnToOrigin();
  }

  public void setAllPIDControllers(double p, double i, double d) {
    System.out.println(p+" "+i+" "+d);
    frontRight.setPidController(p, i, d);
    frontLeft.setPidController(p, i, d);
    backRight.setPidController(p, i, d);
    backLeft.setPidController(p, i, d);
  }

  public SwerveModule[] getRawModules(){
    return new SwerveModule[]{frontLeft, frontRight, backLeft, backRight};
  }
}