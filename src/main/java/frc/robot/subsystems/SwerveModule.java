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
import frc.lib.lib3512.config.SwerveModuleConstants;
import frc.lib.lib3512.math.OnboardModuleState;
import frc.lib.lib3512.util.CANCoderUtil;
import frc.lib.lib3512.util.CANCoderUtil.CCUsage;
import frc.lib.lib3512.util.CANSparkMaxUtil;
import frc.lib.lib3512.util.CANSparkMaxUtil.Usage;
import frc.robot.Config;
import frc.robot.Robot;

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d steerOffset;

  private CANSparkMax steerMotor;
  private CANSparkMax driveMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedSteerEncoder;
  private CANCoder steerEncoder;

  private final SparkMaxPIDController driveController;
  private final SparkMaxPIDController steerController;

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          Config.Swerve.DRIVE_S, Config.Swerve.DRIVE_V, Config.Swerve.DRIVE_A);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    steerOffset = moduleConstants.steerOffset;

    /* Steer Encoder Config */
    steerEncoder = new CANCoder(moduleConstants.cancoderID);
    configSteerEncoder();

    /* Steer Motor Config */
    steerMotor = new CANSparkMax(moduleConstants.steerMotorID, MotorType.kBrushless);
    integratedSteerEncoder = steerMotor.getEncoder();
    steerController = steerMotor.getPIDController();
    configSteerMotor();

    /* Drive Motor Config */
    driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
    configDriveMotor();

    lastAngle = getState().angle;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // Custom optimize command, since default WPILib optimize assumes continuous controller which
    // REV and CTRE are not
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

    setSteer(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  public void stopMotors() {
    driveMotor.stopMotor();
    steerMotor.stopMotor();
  }

  private void resetToAbsolute() {
    double absolutePosition = getCanCoder().getDegrees() - steerOffset.getDegrees();
    integratedSteerEncoder.setPosition(absolutePosition);
  }

  private void configSteerEncoder() {
    steerEncoder.configFactoryDefault();
    CANCoderUtil.setCANCoderBusUsage(steerEncoder, CCUsage.kMinimal);
    steerEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
  }

  private void configSteerMotor() {
    steerMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(steerMotor, Usage.kPositionOnly);
    steerMotor.setSmartCurrentLimit(Config.Swerve.STEER_CURRENT_LIMIT);
    steerMotor.setInverted(Config.Swerve.STEER_INVERT);
    steerMotor.setIdleMode(Config.Swerve.STEER_IDLE_MODE);
    integratedSteerEncoder.setPositionConversionFactor(Config.Swerve.STEER_POS_CONVERSION);
    steerController.setP(Config.Swerve.STEER_P);
    steerController.setI(Config.Swerve.STEER_I);
    steerController.setD(Config.Swerve.STEER_D);
    steerController.setFF(Config.Swerve.STEER_FF);
    steerMotor.enableVoltageCompensation(Config.Swerve.VOLTAGE_COMPENSATION);
    steerMotor.burnFlash();
    resetToAbsolute();
  }

  private void configDriveMotor() {
    driveMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
    driveMotor.setSmartCurrentLimit(Config.Swerve.DRIVE_CURRENT_LIMIT);
    driveMotor.setInverted(Config.Swerve.DRIVE_INVERT);
    driveMotor.setIdleMode(Config.Swerve.DRIVE_IDLE_MODE);
    driveEncoder.setVelocityConversionFactor(Config.Swerve.DRIVE_VEL_CONVERSION);
    driveEncoder.setPositionConversionFactor(Config.Swerve.DRIVE_POS_CONVERSION);
    driveController.setP(Config.Swerve.DRIVE_P);
    driveController.setI(Config.Swerve.DRIVE_I);
    driveController.setD(Config.Swerve.DRIVE_D);
    driveController.setFF(Config.Swerve.DRIVE_FF);
    driveMotor.enableVoltageCompensation(Config.Swerve.VOLTAGE_COMPENSATION);
    driveMotor.burnFlash();
    driveEncoder.setPosition(0.0);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Config.Swerve.MAX_ATTAINABLE_SPEED;
      driveMotor.set(percentOutput);
    } else {
      driveController.setReference(
          desiredState.speedMetersPerSecond,
          ControlType.kVelocity,
          0,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  private void setSteer(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (Config.Swerve.MAX_ATTAINABLE_SPEED * 0.01))
            ? lastAngle
            : desiredState.angle;

    steerController.setReference(angle.getDegrees(), ControlType.kPosition);
    lastAngle = angle;
  }

  private Rotation2d getSteeringAngle() {
    return Rotation2d.fromDegrees(integratedSteerEncoder.getPosition());
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(steerEncoder.getAbsolutePosition());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getSteeringAngle());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveEncoder.getPosition(), getSteeringAngle());
  }
}
