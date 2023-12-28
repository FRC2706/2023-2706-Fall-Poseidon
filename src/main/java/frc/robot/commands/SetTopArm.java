package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmPneumaticsSubsystem;
import frc.robot.subsystems.ArmSubsystem;

public class SetTopArm extends CommandBase{
    double topArmAngleRadians;
    final double TIMEOUT_S=2;

    Timer m_timer = new Timer();
    /*Creates a new SetTopArm */
    public SetTopArm(double angleDegree) {
        topArmAngleRadians = Math.toRadians(angleDegree);
        //addRequirements() used to set subsystem dependencies
        addRequirements(ArmSubsystem.getInstance());
        addRequirements(ArmPneumaticsSubsystem.getInstance());
    }

    // called when command is initially scheduled
    @Override 
    public void initialize() {
        ArmPneumaticsSubsystem.getInstance().controlTopBrake(false, false);
        m_timer.stop();
        m_timer.reset();
        m_timer.start();
    }

    @Override 
    public void execute() {
        ArmSubsystem.getInstance().setTopJointAngle(topArmAngleRadians);
    }

    @Override 
    public void end(boolean interrupted) {
        ArmSubsystem.getInstance().stopMotors();
        if (interrupted == false)
        {
            ArmPneumaticsSubsystem.getInstance().controlTopBrake(true, true);
        }
        m_timer.stop();
    }
    @Override 
    public boolean isFinished() {
        return m_timer.hasElapsed(TIMEOUT_S);
    }
}
