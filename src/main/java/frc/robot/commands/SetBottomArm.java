// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class SetBottomArm extends CommandBase {
  double bottomArmAngleRadians;

  Timer m_timer = new Timer();
  /** Creates a new SetBottomArm. */
  public SetBottomArm(double angleDegree) {
    bottomArmAngleRadians = Math.toRadians(angleDegree);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ArmSubsystem.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ArmSubsystem.getInstance().controlBottomArmBrake(false);
    m_timer.stop();
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ArmSubsystem.getInstance().setBottomJointAngle(bottomArmAngleRadians);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ArmSubsystem.getInstance().stopMotors();
    if (interrupted == false) {
      ArmSubsystem.getInstance().controlBottomArmBrake(true);
    }
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //todo: check if bottom arm reaches the position 
    //time out, finished
    return m_timer.hasElapsed(1);
  }
}
