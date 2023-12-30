// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmPneumaticsSubsystem;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArm extends Command {
  private final double HOLD_TIME_FOR_BRAKE = 0.3;
  private final double m_botPos;
  private final double m_topPos;

  private final Timer m_timer = new Timer();
  private boolean m_timerStarted = false;

  /** Creates a new MoveArm. */
  public MoveArm(double botPos, double topPos) {
    m_botPos = botPos;
    m_topPos = topPos;

    addRequirements(ArmSubsystem.getInstance(), ArmPneumaticsSubsystem.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ArmSubsystem.getInstance().resetProfiledPIDControllers();

    ArmPneumaticsSubsystem.getInstance().controlBottomBrake(true, true);
    ArmPneumaticsSubsystem.getInstance().controlTopBrake(true, true);

    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ArmSubsystem.getInstance().setAngles(m_botPos, m_topPos, 0, 0);

    if (!m_timerStarted && ArmSubsystem.getInstance().isAtSetpoint()) {
      m_timerStarted = true;

      m_timer.restart();
     
      ArmPneumaticsSubsystem.getInstance().controlBottomBrake(false, true);
      ArmPneumaticsSubsystem.getInstance().controlTopBrake(false, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ArmSubsystem.getInstance().stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(HOLD_TIME_FOR_BRAKE);
  }
}
