// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class SyncArmEncoders extends Command {
  int counter;
  boolean bSynced;
  /** Creates a new SyncArmEncoders. */
  public SyncArmEncoders() {
    counter = 0;
    bSynced = false;
    addRequirements(ArmSubsystem.getInstance());
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    counter ++;
    // if (ArmSubsystem.getInstance().areEncodersSynced()==false) {
    //   DriverStation.reportWarning ("Arm encoders are not synced", false);
    //   System.out.println("*****Arm encoders are not synced*****");
    // }
    // else {
    //   DriverStation.reportWarning ("Arm encoders are synced", false);
    //   System.out.println("*****Arm encoders are synced*****");
    //   bSynced = true;
    //   ArmSubsystem.getInstance().burnFlash();
    // }
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  @Override 
  public boolean runsWhenDisabled() {
    return true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (counter >= 10 || bSynced == true) {
    return true;
    }
    else {

      return false;
    }
  }
}
