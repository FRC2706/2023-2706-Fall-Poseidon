// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class SyncArmEncoders extends CommandBase {
  /** Creates a new SyncArmEncoders. */
  public SyncArmEncoders() {
    addRequirements(ArmSubsystem.getInstance());
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (ArmSubsystem.getInstance().areEncodersSynced()==false) {
      DriverStation.reportWarning ("Arm encoders are not synced", false);
      System.out.println("*****Arm encoders are not synced*****");
    }
    else {
      DriverStation.reportWarning ("Arm encoders are synced", false);
      System.out.println("*****Arm encoders are synced*****");

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
