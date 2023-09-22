package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.lib2706.SubsystemChecker;
import frc.lib.lib2706.SubsystemChecker.SubsystemType;
import frc.robot.Config;

public class SwerveSubsystem extends SubsystemBase {
  private static SwerveSubsystem INSTANCE;

  private final PigeonIMU m_pigeon;

  private SwerveDriveOdometry m_odometry;
  private SwerveModule[] m_swerveModules;

  public static SwerveSubsystem getInstance() {
    if (INSTANCE == null) {
      SubsystemChecker.subsystemConstructed(SubsystemType.SwerveSubsystem);
      INSTANCE = new SwerveSubsystem();
    }
    return INSTANCE;
  }

  private SwerveSubsystem() {
    m_pigeon = new PigeonIMU(Config.CanID.PIGEON);
    m_pigeon.configFactoryDefault();

    m_swerveModules = new SwerveModule[] {
        new SwerveModule(0, Config.Swerve.Mod0.CONSTANTS),
        new SwerveModule(1, Config.Swerve.Mod1.CONSTANTS),
        new SwerveModule(2, Config.Swerve.Mod2.CONSTANTS),
        new SwerveModule(3, Config.Swerve.Mod3.CONSTANTS)
    };

    m_odometry = new SwerveDriveOdometry(
        Config.Swerve.SWERVE_KINEMATICS,
        getYaw(),
        getPositions(),
        new Pose2d());
  }

  public void stopMotors() {
    for (SwerveModule mod : m_swerveModules) {
      mod.stopMotors();
    }
  }

  public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates;
    if (fieldRelative) {
      swerveModuleStates = Config.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              xSpeed,
              ySpeed,
              rotSpeed,
              getHeading()));
    } else {
      swerveModuleStates = Config.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(
          new ChassisSpeeds(
              xSpeed,
              ySpeed,
              rotSpeed));
    }
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Config.Swerve.MAX_ATTAINABLE_SPEED);

    for (SwerveModule mod : m_swerveModules) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Config.Swerve.MAX_ATTAINABLE_SPEED);

    for (SwerveModule mod : m_swerveModules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        getYaw(),
        getPositions(),
        pose);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : m_swerveModules) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : m_swerveModules) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  /**
   * Returns the heading of the robot.
   * 
   * This is private because only the odoemtry get's the raw gyro value.
   * Everything else get's the gyro value from odometry since it does an
   * offset.
   *
   * @return the robot's heading as a Rotation2d, from -180 to 180
   */
  private Rotation2d getYaw() {
    return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());
  }

  /**
   * Returns the heading of the robot.
   * 
   * Uses this method for heading. Odometry does an offset to ensure this has the
   * correct origin.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public Rotation2d getHeading() {
    return m_odometry.getPoseMeters().getRotation();
  }

  @Override
  public void periodic() {
    m_odometry.update(getYaw(), getPositions());
  }

  /**
   * Returns a command that resets the heading to the given heading when
   * scheduled.
   * 
   * Maintains the current X and Y value of odometry.
   * 
   * @param newHeading Desired heading to reset to.
   * @return A command to reset odometry
   */
  public CommandBase getResetHeadingCommand(Rotation2d newHeading) {
    return Commands.runOnce(
        () -> resetOdometry(
            new Pose2d(
                getPose().getTranslation(),
                newHeading)));
  }
}
