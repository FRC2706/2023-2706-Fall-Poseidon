package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.PubSubOptions;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.lib2706.AdvantageUtil;
import frc.lib.lib2706.PoseBuffer;
import frc.robot.Config;

public class SwerveSubsystem extends SubsystemBase {
  private final PigeonIMU gyro;

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;
  String tableName = "SwerveChassis";
  private NetworkTable swerveTable = NetworkTableInstance.getDefault().getTable(tableName);
  private DoublePublisher pubCurrentAngle = swerveTable.getDoubleTopic("Current angle (deg)").publish(PubSubOption.periodic(0.02));
  private DoublePublisher pubCurrentPositionX = swerveTable.getDoubleTopic("Current positionX (m) ").publish(PubSubOption.periodic(0.02));
  private DoublePublisher pubCurrentPositionY = swerveTable.getDoubleTopic("Current positionY (m) ").publish(PubSubOption.periodic(0.02));
  private DoubleArrayPublisher pubCurrentPose = swerveTable.getDoubleArrayTopic("Pose ").publish(PubSubOption.periodic(0.02));

  // private DoubleArrayPublisher moduleStatePublisher;
  // private DoubleArrayPublisher moduleStatePublisher;

  
  private Field2d field;

  private PoseBuffer poseBuffer;

  private static SwerveSubsystem instance;
  public static SwerveSubsystem getInstance(){
      if(instance == null){
          instance = new SwerveSubsystem();
      }
      return instance;
  }

  public SwerveSubsystem() {
    gyro = new PigeonIMU(Config.Swerve.pigeonID);
    gyro.configFactoryDefault();
    // zeroGyro();

    

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
      ChassisSpeeds speeds, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
    Config.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative ? 
                ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getYaw()) :
                speeds);
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

  public Rotation2d getYaw() {
    return (Config.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getYaw());
  }
  
  public Command setHeadingCommand(Rotation2d rotation2d) {
    return Commands.runOnce(() -> resetOdometry(new Pose2d(getPose().getTranslation(), rotation2d)));
  }
  public Command setOdometryCommand(Pose2d pose) {
    return Commands.runOnce(() -> resetOdometry(pose));
  }
  public Command lockWheelsInX() {
    return Commands.run(() -> setModuleStates(
      new SwerveModuleState[]{
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(45))
      })
    );
  }

  /**
   * Get a pose at the given timestamp. 
   * Returns an empty Optional if the buffer is empty or doesn't go back far enough.
   * 
   * @param timestampSeconds The timestamp for the pose to get, matching WPILib PoseEstimator's 
   *                         timestamps (which matches PhotonVision and Limelight)
   * @return An Optional of the Pose2d or an empty Optional.
   */
  public Optional<Pose2d> getPoseAtTimestamp(double timestampSeconds) {
    return(poseBuffer.getPoseAtTimestamp(timestampSeconds));
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

    poseBuffer.addPoseToBuffer(getPose());

    pubCurrentAngle.accept(getPose().getRotation().getDegrees());
    pubCurrentPositionX.accept(getPose().getX());
    pubCurrentPositionY.accept(getPose().getY());
    pubCurrentPose.accept(AdvantageUtil.deconstruct(getPose()));


  }
}
