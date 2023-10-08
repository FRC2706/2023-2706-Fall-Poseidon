// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Config;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.lib2706.SubsystemChecker;
import frc.lib.lib2706.SubsystemChecker.SubsystemType;



public class ArmSubsystem extends SubsystemBase {
  private static ArmSubsystem instance = null; //static object that contains all movement controls

  private static final MotorType motorType = MotorType.kBrushless; //defines brushless motortype
  public final CANSparkMax m_bottomArm; //bottom SparkMax motor controller
  public SparkMaxPIDController m_pidControllerTopArm;

  //duty cycle encoder
  private DutyCycleEncoder m_bottomDutyCycleEncoder;

  public static ArmSubsystem getInstance() { //gets arm subsystem object to control movement
    if (instance == null) {
      SubsystemChecker.subsystemConstructed(SubsystemType.ArmSubsystem);
      instance = new ArmSubsystem();
    }
    return instance;
  }

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {

    m_bottomArm = new CANSparkMax(Config.CANID.BOTTOM_ARM_SPARK_CAN_ID, motorType); //creates SparkMax motor controller for bottom joint
    m_bottomArm.restoreFactoryDefaults();
    m_bottomArm.setSmartCurrentLimit(ArmConfig.CURRENT_LIMIT);
    m_bottomArm.setInverted(ArmConfig.BOTTOM_SET_INVERTED); //sets movement direction
    m_bottomArm.setIdleMode(IdleMode.kBrake); //sets brakes when there is no motion
    
    m_bottomArm.setSoftLimit(SoftLimitDirection.kForward, ArmConfig.bottom_arm_forward_limit);
    m_bottomArm.setSoftLimit(SoftLimitDirection.kReverse, ArmConfig.bottom_arm_reverse_limit);
    m_bottomArm.enableSoftLimit(SoftLimitDirection.kForward, ArmConfig.BOTTOM_SOFT_LIMIT_ENABLE);
    m_bottomArm.enableSoftLimit(SoftLimitDirection.kReverse, ArmConfig.BOTTOM_SOFT_LIMIT_ENABLE);

    CANCoderConfiguration config = new CANCoderConfiguration();
    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    config.sensorTimeBase = SensorTimeBase.PerSecond;
    config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    config.magnetOffsetDegrees = 0;
    config.sensorDirection = true;

    m_bottomDutyCycleEncoder = new DutyCycleEncoder(ArmConfig.bottom_duty_cycle_channel);
    m_bottomDutyCycleEncoder.setDistancePerRotation(360);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public double getAbsoluteBottom() {
   // return Math.toRadians(m_bottomDutyCycleEncoder.getAbsolutePosition() * -360 + m_bottomArmOffset.get());
   return 0.0;
  }
}
