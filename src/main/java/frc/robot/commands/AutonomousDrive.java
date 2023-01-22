// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class AutonomousDrive extends CommandBase {
  
  private final SwerveSubsystem driveSubsystem;
  private final SwerveControllerCommand yeet;
  private boolean done = false;

  public AutonomousDrive(SwerveSubsystem swerveSubsystem) {
    this.driveSubsystem = swerveSubsystem;
    addRequirements(driveSubsystem);
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      3,
      1).setKinematics(Constants.kSwerveDriveKinematics);

    // 2. Generate trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, new Rotation2d(0)),
          List.of(
                  new Translation2d(1, 15),
                  new Translation2d(1, -7)),
          new Pose2d(12, -1, Rotation2d.fromDegrees(90)),
          trajectoryConfig);

    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(0.01, 0, 0);
    PIDController yController = new PIDController(0.01, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
          0.01, 0, 0, new Constraints(5/4, Math.PI/4));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    yeet = new SwerveControllerCommand(
      trajectory,
      this.driveSubsystem::getRobotPose,
      Constants.kSwerveDriveKinematics,
      xController,
      yController,
      thetaController,
      this.driveSubsystem::setModuleStates,
      this.driveSubsystem);
  }

  @Override
  public void initialize() {
    this.driveSubsystem.resetRobotPose();
    yeet.schedule();
  }

  @Override
  public void execute() {
    if (yeet.isFinished()){
      done=true;
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return done;
  }
}