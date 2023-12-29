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
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;

public class SwerveSubsystem extends SubsystemBase {
  private final PigeonIMU gyro;

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;
  String tableName = "SwerveChassis";
  private NetworkTable swerveTable = NetworkTableInstance.getDefault().getTable(tableName);
  private DoublePublisher currentAngleEntry = swerveTable.getDoubleTopic("Current angle (deg)").publish();
  private DoublePublisher currentPositionXEntry = swerveTable.getDoubleTopic("Current positionX (m) ").publish();
  private DoublePublisher currentPositionYEntry = swerveTable.getDoubleTopic("Current positionY (m) ").publish();

  // private DoubleArrayPublisher moduleStatePublisher;
  // private DoubleArrayPublisher moduleStatePublisher;

  
  private Field2d field;

  /**
   * Create a SwerveSubsystem. 
   * This should be private and only called by the getInstance method.
   */
  public SwerveSubsystem() {
    gyro = new PigeonIMU(Config.Swerve.pigeonID);
    gyro.configFactoryDefault();
    zeroGyro();

    

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Config.Swerve.Mod0.constants,"FL"),
          new SwerveModule(1, Config.Swerve.Mod1.constants,"FR"),
          new SwerveModule(2, Config.Swerve.Mod2.constants,"BL"),
          new SwerveModule(3, Config.Swerve.Mod3.constants,"BR")
        };

    swerveOdometry = new SwerveDriveOdometry(Config.Swerve.swerveKinematics, getYaw(), getPositions(), new Pose2d() );

    field = new Field2d();
    SmartDashboard.putData("Field", field);
  }

  /**
   * Drive the swerve chassis by the given X speed, Y speed and angular speed given.
   * Can drive relative to the field or relative to the robot.
   * 
   * Speeds are in meters per second (m/s).
   * Angular speed is in radians per second (rad/s).
   * 
   * @param speeds The speeds to drive at, as a {@link ChassisSpeeds} object.
   * @param fieldRelative True for field relative, false for robot relative.
   * @param isOpenLoop True for open loop velocity control, false for closed loop velocity control.
   */
  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
    Config.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Config.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()], isOpenLoop);
    }
  }

  /**
   * Set the states of each module individually.
   * Used by PathPlanner to control the swerve chassis.
   * 
   * The size of the array given must match the number of modules.
   * 
   * @param desiredStates An array of {@link SwerveModuleState}.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Config.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.getModuleNumber()], false);
    }
  }

  /**
   * Get the pose of the robot using odometry.
   * 
   * @return A {@link Pose2d} of where the robot is on the field.
   */
  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  /**
   * Reset odometry to the given pose.
   * 
   * @param pose The {@link Pose2d} to reset odometry to.
   */
  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
  }

  /**
   * Get the {@link SwerveModuleState}s of all modules.
   * Contains the velocity and angle of each module.
   * 
   * @return An array of {@link SwerveModuleState} objects.
   */
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.getModuleNumber()] = mod.getState();
    }
    return states;
  }

  /**
   * Get the {@link SwerveModulePosition}s of all modules.
   * Contains the position and angle of each module.
   * 
   * @return An array of {@link SwerveModulePosition} objects.
   */
  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.getModuleNumber()] = mod.getPosition();
    }
    return positions;
  }

  public void zeroGyro() {
    gyro.setYaw(0);
  }

  public Rotation2d getYaw() {
    return (Config.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getYaw());
  }

  @Override
  public void periodic() {
    SwerveModulePosition[] tempGetPositions = getPositions();
    SwerveModuleState[] tempGetStates = getStates();
    swerveOdometry.update(getYaw(), tempGetPositions);
    field.setRobotPose(getPose());

    for (SwerveModule mod : mSwerveMods) {
      mod.periodic();
    }

    currentAngleEntry.accept(getPose().getRotation().getDegrees());
    currentPositionXEntry.accept(getPose().getX());
    currentPositionYEntry.accept(getPose().getY());
  }

  public Command getResetOdometryCommand(Pose2d pose) {
    return Commands.runOnce(() -> resetOdometry(pose));
  }
}
