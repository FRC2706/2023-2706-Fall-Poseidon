// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmPneumaticsSubsystem;
import frc.robot.subsystems.ArmSubsystem;

public class ControlSingleArm extends Command {
    private boolean m_doTopNotBot;
    private double m_angleSetpointRad;

    /** 
     * Creates a new ControlSingleArm. 
     */
    public ControlSingleArm(boolean doTopNotBottom, double angleSetpointRad) {
        m_doTopNotBot = doTopNotBottom;
        m_angleSetpointRad = angleSetpointRad;

        addRequirements(ArmSubsystem.getInstance(), ArmPneumaticsSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        ArmSubsystem.getInstance().resetProfiledPIDControllers();

        if (m_doTopNotBot) {
            ArmPneumaticsSubsystem.getInstance().controlTopBrake(false, true);
        } else {
            ArmPneumaticsSubsystem.getInstance().controlBottomBrake(false, true);
        }

        // Hack to update ArmDisplay for controlling 1 motor
        if (m_doTopNotBot) {
            ArmSubsystem.getInstance().setAngles(
                ArmSubsystem.getInstance().getBotPosition(), 
                m_angleSetpointRad, 
                0, 0);
        } else {
            ArmSubsystem.getInstance().setAngles(
                m_angleSetpointRad, 
                ArmSubsystem.getInstance().getTopPosition(), 
                0, 0);
        }
        ArmSubsystem.getInstance().stopMotors();
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_doTopNotBot) {
            ArmSubsystem.getInstance().setTopAngle(m_angleSetpointRad, 0);
        } else {
            ArmSubsystem.getInstance().setBotAngle(m_angleSetpointRad, 0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        ArmSubsystem.getInstance().stopMotors();

        if (m_doTopNotBot) {
            ArmPneumaticsSubsystem.getInstance().controlTopBrake(true, true);
        } else {
            ArmPneumaticsSubsystem.getInstance().controlBottomBrake(true, true);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
