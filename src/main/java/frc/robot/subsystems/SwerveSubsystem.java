package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase {
  private final PigeonIMU m_pigeon;

  private SwerveDriveOdometry m_odometry;
  private SwerveModule[] m_swerveModules;

  public SwerveSubsystem() {
    m_pigeon = new PigeonIMU(Constants.Swerve.pigeonID);
    m_pigeon.configFactoryDefault();

    m_swerveModules = new SwerveModule[] {
        new SwerveModule(0, Constants.Swerve.Mod0.constants),
        new SwerveModule(1, Constants.Swerve.Mod1.constants),
        new SwerveModule(2, Constants.Swerve.Mod2.constants),
        new SwerveModule(3, Constants.Swerve.Mod3.constants)
    };

    m_odometry = new SwerveDriveOdometry(
        Constants.Swerve.swerveKinematics,
        getYaw(),
        getPositions(),
        new Pose2d());
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getYaw())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : m_swerveModules) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

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

  public void resetHeading(Rotation2d newHeading) {
    resetOdometry(
        new Pose2d(
            getPose().getTranslation(),
            newHeading));
  }

  @Override
  public void periodic() {
    m_odometry.update(getYaw(), getPositions());
  }

  /**
   * Returns a command that resets the heading to the given heading when scheduled.
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
