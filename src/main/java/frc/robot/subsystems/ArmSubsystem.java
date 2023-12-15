// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.lib2706.ErrorCheck;
import frc.lib.lib2706.SubsystemChecker;
import frc.lib.lib2706.SubsystemChecker.SubsystemType;
import frc.robot.Config;
import frc.robot.Config.CANID;

public class ArmSubsystem extends SubsystemBase {
  private static ArmSubsystem instance = null;

  private final CANSparkMax m_botSpark, m_topSpark;
  private final SparkMaxPIDController m_botSparkPid, m_topSparkPid;
  private final AbsoluteEncoder m_botEncoder, m_topEncoder;
  
  private final ProfiledPIDController m_botPid, m_topPid;
  
  public static ArmSubsystem getInstance() {
    if (instance == null) {
      SubsystemChecker.subsystemConstructed(SubsystemType.ArmSubsystem);
      instance = new ArmSubsystem();
    }
    return instance;
  }

  /** 
   * Creates a new ArmSubsystem. 
   */
  public ArmSubsystem() {
    /**
     * Setup CANSparkMaxs
     */
    m_botSpark = new CANSparkMax(CANID.BOTTOM_ARM_SPARK_CAN_ID, Config.NEO_MOTORTYPE);
    m_topSpark = new CANSparkMax(CANID.TOP_ARM_SPARK_CAN_ID, Config.NEO_MOTORTYPE);

    ErrorCheck.errREV(m_botSpark.restoreFactoryDefaults());
    ErrorCheck.errREV(m_topSpark.restoreFactoryDefaults());

    ErrorCheck.errREV(m_botSpark.setSmartCurrentLimit(ArmConfig.BOT_CURRENT_LIMIT));
    ErrorCheck.errREV(m_topSpark.setSmartCurrentLimit(ArmConfig.TOP_CURRENT_LIMIT));

    m_botSpark.setInverted(ArmConfig.BOT_SET_INVERTED);
    m_topSpark.setInverted(ArmConfig.TOP_SET_INVERTED);

    ErrorCheck.errREV(m_botSpark.setIdleMode(IdleMode.kBrake));
    ErrorCheck.errREV(m_topSpark.setIdleMode(IdleMode.kBrake));
    
    ErrorCheck.errREV(m_botSpark.setSoftLimit(SoftLimitDirection.kForward, ArmConfig.BOT_FORW_LIMIT));
    ErrorCheck.errREV(m_botSpark.setSoftLimit(SoftLimitDirection.kReverse, ArmConfig.BOT_REV_LIMIT));
    ErrorCheck.errREV(m_botSpark.enableSoftLimit(SoftLimitDirection.kForward, ArmConfig.BOT_SOFT_LIMIT_ENABLE));
    ErrorCheck.errREV(m_botSpark.enableSoftLimit(SoftLimitDirection.kReverse, ArmConfig.BOT_SOFT_LIMIT_ENABLE));

    ErrorCheck.errREV(m_topSpark.setSoftLimit(SoftLimitDirection.kForward, ArmConfig.TOP_FORW_LIMIT));
    ErrorCheck.errREV(m_topSpark.setSoftLimit(SoftLimitDirection.kReverse, ArmConfig.TOP_REV_LIMIT));
    ErrorCheck.errREV(m_topSpark.enableSoftLimit(SoftLimitDirection.kForward, ArmConfig.TOP_SOFT_LIMIT_ENABLE));
    ErrorCheck.errREV(m_topSpark.enableSoftLimit(SoftLimitDirection.kReverse, ArmConfig.TOP_SOFT_LIMIT_ENABLE));

    /**
     * Setup Absolute Encoders
     */
    m_botEncoder = m_botSpark.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    ErrorCheck.errREV(m_botEncoder.setPositionConversionFactor(ArmConfig.BOT_POS_CONV_FACTOR));
    ErrorCheck.errREV(m_botEncoder.setVelocityConversionFactor(ArmConfig.BOT_VEL_CONV_FACTOR));
    ErrorCheck.errREV(m_botEncoder.setInverted(ArmConfig.BOT_ENC_INVERT));

    m_topEncoder = m_topSpark.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    ErrorCheck.errREV(m_topEncoder.setPositionConversionFactor(ArmConfig.TOP_POS_CONV_FACTOR));
    ErrorCheck.errREV(m_topEncoder.setVelocityConversionFactor(ArmConfig.TOP_VEL_CONV_FACTOR));
    ErrorCheck.errREV(m_topEncoder.setInverted(ArmConfig.TOP_ENC_INVERT));

    /**
     * Setup SparkMaxPidControllers
     */
    m_botSparkPid = m_botSpark.getPIDController();
    m_topSparkPid = m_topSpark.getPIDController();

    m_botSparkPid.setFeedbackDevice(m_botEncoder);
    m_topSparkPid.setFeedbackDevice(m_topEncoder);

    /**
     * Setup ProfiledPidControllers
     */
    m_botPid = new ProfiledPIDController(
      ArmConfig.BOT_KP, ArmConfig.BOT_KI, ArmConfig.BOT_KD, 
      new Constraints(ArmConfig.BOT_MAX_VEL, ArmConfig.BOT_MAX_ACCEL));

    m_topPid = new ProfiledPIDController(
      ArmConfig.TOP_KP, ArmConfig.TOP_KI, ArmConfig.TOP_KD, 
      new Constraints(ArmConfig.TOP_MAX_VEL, ArmConfig.TOP_MAX_ACCEL));

    /**
     * Burn flash on CANSparkMaxs
     */
    burnFlash();
  }

  @Override
  public void periodic() {
    // double bottomPosition = m_bottomEncoder.getPosition();

    // m_bottomArmPosPub.accept(Math.toDegrees(bottomPosition));
    // m_bottomArmVelPub.accept(m_bottomEncoder.getVelocity());
    // m_bottomAbsoluteEncoder.accept(Math.toDegrees(getAbsoluteBottom()));

  }

  //input angle_bottom in radians
  public void setBottomJointAngle(double angle_bottom) {
    if (angle_bottom<Math.toRadians(0) || angle_bottom>Math.toRadians(95)) {
      angle_bottom = Math.toRadians(95);
    }
    //setReference angle is in radians)
    //todo: tune FF 
    m_pidControllerBottomArm.setReference((angle_bottom), ControlType.kPosition, 0,0.1);
  }


  /**
   * Get the bottom position in radians.
   * 
   * @return Position in radians.
   */
  public double getBotPosition() {
    return m_botEncoder.getPosition();
  }

  /**
   * Get the bottom velocity in radians per second.
   * 
   * @return Velocity in rad/s
   */
  public double getBotVel() {
    return m_botEncoder.getVelocity();
  }

  /**
   * Get the top position in radians.
   * 
   * @return Position in radians.
   */
  public double getTopPosition() {
    return m_topEncoder.getPosition();
  }

  /**
   * Get the top velocity in radians per second.
   * 
   * @return Velocity in rad/s
   */
  public double getTopVel() {
    return m_topEncoder.getVelocity();
  }

  public void stopMotors() {
    m_botSpark.stopMotor();
    m_topSpark.stopMotor();
  }

  /**
   * Burn flash after a 0.2 second delay to ensure settings are saved.
   */
  public void burnFlash() {
    try
    {
      Thread.sleep(200);
    } catch (Exception e) {}

    ErrorCheck.errREV(m_botSpark.burnFlash());
    ErrorCheck.errREV(m_topSpark.burnFlash());
  }

}
