// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.WestCoastDrive;

public class DriveWithJoySticks extends CommandBase {

  private CommandJoystick leftJoyStick;
  private CommandJoystick rightJoyStick;
  private WestCoastDrive westCoastDrive;


  /** Creates a new DriveWithJoySticks. */
  public DriveWithJoySticks(CommandJoystick lJoystick, CommandJoystick rJoystick, WestCoastDrive wCoastDrive) {
    leftJoyStick = lJoystick;
    rightJoyStick = rJoystick;
    westCoastDrive = wCoastDrive;
    addRequirements(westCoastDrive);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    westCoastDrive.drive(leftJoyStick.getY(),rightJoyStick.getY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
