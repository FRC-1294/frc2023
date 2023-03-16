package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmControlSubsystem;

public class ArmHold extends CommandBase {

  private final ArmControlSubsystem _armSystem;  
  public ArmHold(ArmControlSubsystem armSystem) {
    _armSystem = armSystem;    
    addRequirements(_armSystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    _armSystem.intakeHold();
    _armSystem.holdArm();
  }

  @Override
  public void end(boolean interrupted) {
    _armSystem.intakeStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
