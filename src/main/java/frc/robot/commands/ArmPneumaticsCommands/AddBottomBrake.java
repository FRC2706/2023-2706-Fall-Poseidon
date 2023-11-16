package frc.robot.commands.ArmPneumaticsCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmPneumaticsSubsystem;

public class AddBottomBrake extends CommandBase {
  /** Creates a new ArmPneumaticsCommands. */
  public AddBottomBrake() {
    addRequirements(ArmPneumaticsSubsystem.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ArmPneumaticsSubsystem.getInstance().controlBottomBrake(true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
