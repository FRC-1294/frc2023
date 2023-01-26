// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmControlSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoArmIK extends CommandBase {
  //goal
  //calculate a robot movement, arm angle, and arm extend in order to reach an object
  //with the object position as the input

  Pose3d desiredArmPose;

  SwerveSubsystem swervee;
  ArmControlSubsystem armee;

  

  public AutoArmIK(SwerveSubsystem swervee, ArmControlSubsystem armee) {
    this.swervee = swervee;
    this.armee = armee;

    addRequirements(swervee, armee);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
