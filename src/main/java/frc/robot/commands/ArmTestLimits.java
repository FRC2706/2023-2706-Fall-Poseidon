// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmPneumaticsSubsystem;
import frc.robot.subsystems.ArmSubsystem;

public class ArmTestLimits extends Command {
  private final double MAX_EXTRA_VOLTS = 3;
  private final double MAX_MOVEMENT = Math.toRadians(8);
  private CommandXboxController m_joystick;
  private double m_maxExtraVolts;
  private double initalPosition;

  /** Creates a new ArmFFTestCommand. */
  public ArmTestLimits(CommandXboxController joystick) {
    this.m_joystick = joystick;
    this.m_maxExtraVolts = MAX_EXTRA_VOLTS;
    addRequirements(ArmSubsystem.getInstance(), ArmPneumaticsSubsystem.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ArmPneumaticsSubsystem.getInstance().controlTopBrake(true, true);
    ArmPneumaticsSubsystem.getInstance().controlBottomBrake(false, true);
    initalPosition = ArmSubsystem.getInstance().getBotPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double joystickValueTop = m_joystick.getRawAxis(XboxController.Axis.kLeftY.value) * -1;
    joystickValueTop = MathUtil.applyDeadband(joystickValueTop, 0.15);

    ArmSubsystem.getInstance().testFeedForwardBot(joystickValueTop * m_maxExtraVolts);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ArmSubsystem.getInstance().stopMotors();
    ArmPneumaticsSubsystem.getInstance().controlBottomBrake(true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(initalPosition - ArmSubsystem.getInstance().getBotPosition()) > MAX_MOVEMENT;
  }
}
