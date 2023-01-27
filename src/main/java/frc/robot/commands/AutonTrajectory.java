// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class AutonTrajectory extends CommandBase {
  /** Creates a new Trajectory. */

;
  public Trajectory path = new Trajectory();
  public SwerveSubsystem swerve;
  public Pose2d initPose;
  public Transform2d transform;
  public int idx;
  public double deadZone;
  public ChassisSpeeds speeds;
  public moveTo move;
  public static String curvePath = "path/sussybakacurve.wpilib.json";


  public AutonTrajectory(SwerveSubsystem swerve) {
    this.swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    Path trajPath = Filesystem.getDeployDirectory().toPath().resolve(curvePath);
    try {
    path = TrajectoryUtil.fromPathweaverJson(trajPath);
    } catch (IOException e) {}

    initPose = swerve.getRobotPose();
    transform = path.getInitialPose().minus(initPose);
    idx = 0;

    deadZone = Constants.deadZone;
  }

    public Pose2d getPose(int idx, double heading) {
        //Getting pose and applying transformation
        Pose2d desiredPose = path.getStates().get(idx).poseMeters.plus(transform);


        //double totalTime = path.getStates().get(path.getStates().size()-1).timeSeconds;
        // Calculating wanted heading
        double desiredHeading = 0;

        return new Pose2d(desiredPose.getX(), desiredPose.getY(), new Rotation2d(Units.degreesToRadians(desiredHeading)));

    }

    public void getSpeeds() {
        Pose2d currentPose = swerve.getRobotPose();
        Pose2d desiredPose = getPose(idx, currentPose.getRotation().getDegrees());        

        Transform2d distance = desiredPose.minus(currentPose);
    
        move = new moveTo(distance, swerve);
        move.schedule();

        idx ++;

    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (move.isFinished()){
      getSpeeds();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  if (idx>=path.getStates().size()-1) {return true;}
    return false;
  }
}
