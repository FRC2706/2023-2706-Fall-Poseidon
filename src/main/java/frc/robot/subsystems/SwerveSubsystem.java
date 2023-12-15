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
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Config.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
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
    NetworkTableInstance.getDefault().flush();
  }
}
