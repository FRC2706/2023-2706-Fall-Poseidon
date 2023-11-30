package frc.robot.commands;

//old stuff
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Config;
import frc.robot.subsystems.SwerveSubsystem;

public class TeleopSwerve extends CommandBase {

  //least years code
  private SwerveSubsystem s_Swerve;

  private CommandXboxController m_joystick;
  

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(2.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(2.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(4.0 * Math.PI);
  
  

  public TeleopSwerve(SwerveSubsystem s_Swerve, CommandXboxController joystick) {
    this.s_Swerve = s_Swerve;
    this.m_joystick = joystick;
    addRequirements(s_Swerve);

   
  }
  
  @Override
  public void execute() {
    
    /* Get Values and apply deadband to limit unwanted movement*/
    double x = -1 * m_joystick.getRawAxis(XboxController.Axis.kLeftX.value);

    double y = -1 * m_joystick.getRawAxis(XboxController.Axis.kLeftY.value);

    double rot = -1 * m_joystick.getRawAxis(XboxController.Axis.kRightY.value);


    //other teams
    // Get direction and magnitude of linear axes
    double linearMagnitude = Math.hypot(x, y);
    Rotation2d linearDirection = new Rotation2d(x, y);

    // Apply deadband
    linearMagnitude = MathUtil.applyDeadband(linearMagnitude,0.1);
    rot = MathUtil.applyDeadband(rot, 0.1);

    // Apply squaring
    linearMagnitude = Math.copySign(linearMagnitude * linearMagnitude, linearMagnitude);
    rot = Math.copySign(rot * rot, rot);

  
    // Calcaulate new linear components
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .getTranslation();

    // Convert to meters per second
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            translationLimiter.calculate(linearVelocity.getX() * Config.drive_constant),
            strafeLimiter.calculate(linearVelocity.getY() * Config.drive_constant),
            rotationLimiter.calculate(rot * Config.angular));
    
    s_Swerve.drive(speeds,true, true);

  }
}