// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Limelight.Pipeline;


public class LimelightAlign extends CommandBase {
  /** Creates a new CubeAlign. */
  public SwerveSubsystem swerve;
  public Limelight lime;
  public double yaw;
  public int index;
  public double offset;
  public double distance;

  /**
   * Aligns to the nearest target that limelight can dectect
   * @param swervesub the swerve subsystem
   * @param limelightsub the limelight subsystem
   * @param pipelineIndex the index of the pipeline you want limelight to detect
   * @param Xoffset the Xoffset
   * @param distancefromObject the distance you want the robot to be from the object in meters
   */
  public LimelightAlign(SwerveSubsystem swervesub, Limelight limelightsub, int pipelineIndex, double Xoffset, double distanceFromObject) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerve = swervesub;
    lime = limelightsub;
    addRequirements(swerve);
    addRequirements(lime);
    int index = PipelineIndex;
    offset = Xoffset;
    distance = distanceFromObject
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lime.setPipeline(index);
    if (lime.img.hasTargets()) {
      yaw = lime.getTargetYaw();
      double distanceToTravel = lime.getForwardDistance() - distance;
      swerve.setMotors(0, distanceToTravel, yaw*(Math.PI/180)+offset);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (lime.getTargetYaw() <= 1 || lime.getTargetYaw() >= -1) {
      return true;
    }
    return false;
  }
}
