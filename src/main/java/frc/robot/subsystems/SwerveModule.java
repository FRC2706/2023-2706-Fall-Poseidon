package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.lib.lib3512.config.SwerveModuleConstants;
import frc.lib.lib3512.math.OnboardModuleState;
import frc.lib.lib3512.util.CANCoderUtil;
import frc.lib.lib3512.util.CANSparkMaxUtil;
import frc.lib.lib3512.util.CANCoderUtil.CCUsage;
import frc.lib.lib3512.util.CANSparkMaxUtil.Usage;
import frc.robot.Config;
import frc.robot.Robot;

public class SwerveModule {

  private NetworkTable swerveModuleTable;
  private DoublePublisher currentSpeedEntry;
  private DoublePublisher currentAngleEntry;
  private DoublePublisher speedError;
  private DoublePublisher angleError;
  private DoublePublisher desiredSpeedEntry;
  private DoublePublisher desiredAngleEntry;

  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d angleOffset;

  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANCoder angleEncoder;

  private final SparkMaxPIDController driveController;
  private final SparkMaxPIDController angleController;

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
        Config.Swerve.driveKS, Config.Swerve.driveKV, Config.Swerve.driveKA);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants, String ModuleName) {
    this.moduleNumber = moduleNumber;


    angleOffset = moduleConstants.angleOffset;

    String tableName = "SwerveChassis/SwerveModule" + ModuleName;
    swerveModuleTable = NetworkTableInstance.getDefault().getTable(tableName);
  
    /* Angle Encoder Config */
    angleEncoder = new CANCoder(moduleConstants.cancoderID);
    configAngleEncoder();

    /* Angle Motor Config */
    angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getPIDController();
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
    configDriveMotor();

    lastAngle = getState().angle;

    // NetworkTableInstance inst = NetworkTableInstance.getDefault();
    // NetworkTable swerveModuleTable = inst.getTable("datatable");

    desiredSpeedEntry = swerveModuleTable.getDoubleTopic("Desired speed (mps)").publish();
    desiredAngleEntry = swerveModuleTable.getDoubleTopic("Desired angle (deg)").publish();
    currentSpeedEntry = swerveModuleTable.getDoubleTopic("Current speed (mps)").publish();
    currentAngleEntry = swerveModuleTable.getDoubleTopic("Current angle (deg)").publish();
    speedError = swerveModuleTable.getDoubleTopic("Speed error (mps)").publish();
    angleError = swerveModuleTable.getDoubleTopic("Angle error (deg)").publish();

    burnFlash();
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // Custom optimize command, since default WPILib optimize assumes continuous controller which
    // REV and CTRE are not


    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
    desiredState.speedMetersPerSecond *= desiredState.angle.minus(getAngle()).getCos();


    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);


    desiredAngleEntry.accept(desiredState.angle.getRadians());
    desiredSpeedEntry.accept(desiredState.speedMetersPerSecond);
    speedError.accept((desiredState.speedMetersPerSecond)-(driveEncoder.getVelocity()));
    angleError.accept((desiredState.angle.getRadians())-(getAngle().getRadians()));
    NetworkTableInstance.getDefault().flush();
  }

  /**
   * Resets Position Encoder
   */
  private void resetToAbsolute() {
    double absolutePosition = getCanCoder().getRadians() - angleOffset.getRadians();
    integratedAngleEncoder.setPosition(absolutePosition);
  }

  private void configAngleEncoder() {
    angleEncoder.configFactoryDefault();
    CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
    angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
  }

  private void configAngleMotor() {
    angleMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    angleMotor.setSmartCurrentLimit(Config.Swerve.angleContinuousCurrentLimit);
    angleMotor.setInverted(Config.Swerve.angleInvert);
    angleMotor.setIdleMode(Config.Swerve.angleNeutralMode);
    integratedAngleEncoder.setPositionConversionFactor(Config.Swerve.angleConversionFactor);
    angleController.setP(Config.Swerve.angleKP);
    angleController.setI(Config.Swerve.angleKI);
    angleController.setD(Config.Swerve.angleKD);
    angleController.setFF(Config.Swerve.angleKFF);
    angleMotor.enableVoltageCompensation(Config.Swerve.voltageComp);
    resetToAbsolute();
    angleMotor.burnFlash();
  }

  private void configDriveMotor() {
    driveMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
    driveMotor.setSmartCurrentLimit(Config.Swerve.driveContinuousCurrentLimit);
    driveMotor.setInverted(Config.Swerve.driveInvert);
    driveMotor.setIdleMode(Config.Swerve.driveNeutralMode);
    driveEncoder.setVelocityConversionFactor(Config.Swerve.driveConversionVelocityFactor);
    driveEncoder.setPositionConversionFactor(Config.Swerve.driveConversionPositionFactor);
    driveController.setP(Config.Swerve.angleKP);
    driveController.setI(Config.Swerve.angleKI);
    driveController.setD(Config.Swerve.angleKD);
    driveController.setFF(Config.Swerve.angleKFF);
    driveMotor.enableVoltageCompensation(Config.Swerve.voltageComp);
    driveMotor.burnFlash();
    driveEncoder.setPosition(0.0);
  }

  /**
   * Save the configurations from flash to EEPROM.
   */
  public void burnFlash() {
    try {
      Thread.sleep(200);
    } 
    catch (Exception e) {}

    driveMotor.burnFlash();
    angleMotor.burnFlash();
  }

  /*
   * Sets Speed
   * 
   * @param desiredState
   * @param isOpenLoop Op
   */
  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Config.Swerve.maxSpeed;
      driveMotor.set(percentOutput);
    } else {
      driveController.setReference(
          desiredState.speedMetersPerSecond,
          ControlType.kVelocity,
          0,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  /**
   * Sets Angle
   * 
   * @param desiredState 
   */
  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (Config.Swerve.maxSpeed * 0.01))
            ? lastAngle
            : desiredState.angle;

    angleController.setReference(angle.getRadians(), ControlType.kPosition);
    lastAngle = angle;
  }

  /**
   * Returns Angle
   * 
   * @return An
   */
  private Rotation2d getAngle() {
    return (new Rotation2d(integratedAngleEncoder.getPosition()));
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
  }

  public void periodic() {
    //update network tables

    currentSpeedEntry.accept(driveEncoder.getVelocity());
    currentAngleEntry.accept(getAngle().getRadians());
    NetworkTableInstance.getDefault().flush();
  }
}
