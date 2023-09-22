package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class TeleopSwerveCommand extends CommandBase {
  private SwerveSubsystem s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;

  public TeleopSwerveCommand(
      SwerveSubsystem s_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
  }

  @Override
  public void execute() {
    /* Get Values, Deadband */
    double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Swerve.JOYSTICK_DEADBAND);
    double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Swerve.JOYSTICK_DEADBAND);
    double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Swerve.JOYSTICK_DEADBAND);

    translationVal *= Constants.Swerve.TELEOP_SPEED;
    strafeVal *= Constants.Swerve.TELEOP_SPEED;
    rotationVal *= Constants.Swerve.TELEOP_ANGULAR_SPEED;

    /* Drive */
    s_Swerve.drive(
        translationVal, strafeVal,
        rotationVal,
        true,
        true);
  }
}
