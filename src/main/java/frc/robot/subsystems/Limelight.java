// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  public Limelight() {
    table.getEntry("pipeline").setNumber(0);
  }  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public double getHorizontal() {
    return table.getEntry("thor").getDouble(0);
  }
  
  public double getVertical() {
    return table.getEntry("tvert").getDouble(0);
  }

  public double getTX() {
    return table.getEntry("tx").getDouble(0);
  }

  public double getTY() {
    return table.getEntry("ty").getDouble(0);
  }

  public boolean getDetected() {
    if ((table.getEntry("tv").getDouble(0)) == 1) return true;
    return false;
  }
}
