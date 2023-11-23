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
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Config;
import frc.robot.ErrorCheck;
import frc.robot.SubsystemChecker;
import frc.robot.SubsystemChecker.SubsystemType;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  private static ArmSubsystem instance = null; //static object that contains all movement controls

  private static final MotorType motorType = MotorType.kBrushless; //defines brushless motortype
  public final CANSparkMax m_bottomArm; //bottom SparkMax motor controller
  
  //network table entry
  private final String m_tuningTableBottom = "Arm/BottomArmTuning";
  private final String m_dataTableBottom = "Arm/BottomArmData";
  
  //network table entries for bottom arm
  private DoubleEntry m_bottomArmPSubs;
  private DoubleEntry m_bottomArmISubs;
  private DoubleEntry m_bottomArmDSubs;
  private DoubleEntry m_bottomArmIzSubs;
  private DoubleEntry m_bottomArmFFSubs;
  private DoublePublisher m_bottomArmSetpointPub;   
  private DoublePublisher m_bottomArmVelPub;
  private DoubleEntry m_bottomArmMomentToVoltage;
  private DoublePublisher m_bottomArmFFTestingVolts;
  private DoubleEntry m_bottomArmOffset;
  private DoublePublisher m_bottomAbsoluteEncoder;
  private DoublePublisher m_bottomArmPosPub;

  //for bottom arm ff
  private DoubleSubscriber momentToVoltageConversion;
  private double m_bottomVoltageConversion;

  //duty cycle encoder
  private DutyCycleEncoder m_bottomDutyCycleEncoder;

  //embedded relative encoder
  private RelativeEncoder m_bottomEncoder;
  public SparkMaxPIDController m_pidControllerBottomArm;  

  //for arm pneumatic brakakes 
  //DoubleSolenoid brakeSolenoidLow;

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
    ErrorCheck.errREV(m_bottomArm.restoreFactoryDefaults());
    ErrorCheck.errREV(m_bottomArm.setSmartCurrentLimit(ArmConfig.CURRENT_LIMIT));
    m_bottomArm.setInverted(ArmConfig.BOTTOM_SET_INVERTED); //sets movement direction
    ErrorCheck.errREV(m_bottomArm.setIdleMode(IdleMode.kBrake)); //sets brakes when there is no motion
    
    ErrorCheck.errREV(m_bottomArm.setSoftLimit(SoftLimitDirection.kForward, ArmConfig.bottom_arm_forward_limit));
    ErrorCheck.errREV(m_bottomArm.setSoftLimit(SoftLimitDirection.kReverse, ArmConfig.bottom_arm_reverse_limit));
    ErrorCheck.errREV(m_bottomArm.enableSoftLimit(SoftLimitDirection.kForward, ArmConfig.BOTTOM_SOFT_LIMIT_ENABLE));
    ErrorCheck.errREV(m_bottomArm.enableSoftLimit(SoftLimitDirection.kReverse, ArmConfig.BOTTOM_SOFT_LIMIT_ENABLE));

    m_bottomDutyCycleEncoder = new DutyCycleEncoder(ArmConfig.bottom_duty_cycle_channel);

    m_bottomEncoder = m_bottomArm.getEncoder();
    //position in radius
    ErrorCheck.errREV(m_bottomEncoder.setPositionConversionFactor(ArmConfig.bottomArmPositionConversionFactor));
    ErrorCheck.errREV(m_bottomEncoder.setVelocityConversionFactor(ArmConfig.bottomArmVelocityConversionFactor));

    m_pidControllerBottomArm = m_bottomArm.getPIDController();


    NetworkTable bottomArmTuningTable = NetworkTableInstance.getDefault().getTable(m_tuningTableBottom);
    m_bottomArmPSubs = bottomArmTuningTable.getDoubleTopic("P").getEntry(ArmConfig.bottom_arm_kP);
    m_bottomArmISubs = bottomArmTuningTable.getDoubleTopic("I").getEntry(ArmConfig.bottom_arm_kI);
    m_bottomArmDSubs = bottomArmTuningTable.getDoubleTopic("D").getEntry(ArmConfig.bottom_arm_kD);
    m_bottomArmIzSubs = bottomArmTuningTable.getDoubleTopic("IZone").getEntry(ArmConfig.bottom_arm_kIz);
    m_bottomArmFFSubs = bottomArmTuningTable.getDoubleTopic("FF").getEntry(ArmConfig.bottom_arm_kFF);
    m_bottomArmOffset = bottomArmTuningTable.getDoubleTopic("Offset").getEntry(ArmConfig.bottom_arm_offset);
    momentToVoltageConversion = bottomArmTuningTable.getDoubleTopic("VoltageConversion").subscribe(m_bottomVoltageConversion);

    m_bottomArmFFSubs.accept(ArmConfig.bottom_arm_kFF);
    m_bottomArmPSubs.accept(ArmConfig.bottom_arm_kP);
    m_bottomArmISubs.accept(ArmConfig.bottom_arm_kI);
    m_bottomArmDSubs.accept(ArmConfig.bottom_arm_kD);
    m_bottomArmIzSubs.accept(ArmConfig.bottom_arm_kIz);
    m_bottomArmOffset.accept(ArmConfig.bottom_arm_offset);

    m_bottomArmOffset = bottomArmTuningTable.getDoubleTopic("Offset").getEntry(ArmConfig.bottom_arm_offset);
    m_bottomArmOffset.accept(ArmConfig.bottom_arm_offset);

    NetworkTable bottomArmDataTable = NetworkTableInstance.getDefault().getTable(m_dataTableBottom);
    
    m_bottomArmPosPub = bottomArmDataTable.getDoubleTopic("MeasuredAngle").publish(PubSubOption.periodic(0.02));
    m_bottomAbsoluteEncoder = bottomArmDataTable.getDoubleTopic("Absolute Encoder").publish(PubSubOption.periodic(0.02));
    m_bottomArmVelPub = bottomArmDataTable.getDoubleTopic("Vel").publish(PubSubOption.periodic(0.02));

    ErrorCheck.errREV(m_pidControllerBottomArm.setFF(m_bottomArmFFSubs.get()));
    ErrorCheck.errREV(m_pidControllerBottomArm.setP(m_bottomArmPSubs.get()));
    ErrorCheck.errREV(m_pidControllerBottomArm.setI(m_bottomArmISubs.get()));
    ErrorCheck.errREV(m_pidControllerBottomArm.setD(m_bottomArmDSubs.get()));
    ErrorCheck.errREV(m_pidControllerBottomArm.setIZone(m_bottomArmIzSubs.get())); 
    ErrorCheck.errREV(m_pidControllerBottomArm.setOutputRange(ArmConfig.min_output, ArmConfig.max_output));

    /*brakeSolenoidLow = new DoubleSolenoid(Config.CTRE_PCM_CAN_ID,
                                          PneumaticsModuleType.CTREPCM,
                                          Config.ARMLOW_PNEUMATIC_FORWARD_CHANNEL,
                                          Config.ARMLOW_PNEUMATIC_REVERSE_CHANNEL);
                                          */
    //to do: could be moved to another spot
    updatePIDSettings();
    updateFromAbsoluteBottom();
  }

  public void updatePIDSettings() {
    ErrorCheck.errREV(m_pidControllerBottomArm.setFF(m_bottomArmFFSubs.get()));
    ErrorCheck.errREV(m_pidControllerBottomArm.setP(m_bottomArmPSubs.get()));
    ErrorCheck.errREV(m_pidControllerBottomArm.setI(m_bottomArmISubs.get()));
    ErrorCheck.errREV(m_pidControllerBottomArm.setD(m_bottomArmDSubs.get()));
    ErrorCheck.errREV(m_pidControllerBottomArm.setIZone(m_bottomArmIzSubs.get()));
    ErrorCheck.errREV(m_pidControllerBottomArm.setOutputRange(ArmConfig.min_output, ArmConfig.max_output));
  }

  @Override
  public void periodic() {
    double bottomPosition = m_bottomEncoder.getPosition();

    m_bottomArmPosPub.accept(Math.toDegrees(bottomPosition));
    m_bottomArmVelPub.accept(m_bottomEncoder.getVelocity());
    m_bottomAbsoluteEncoder.accept(Math.toDegrees(getAbsoluteBottom()));

    // This method will be called once per scheduler run
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

  /*public void controlBottomArmBrake( boolean bBrakeOn) {
    if (bBrakeOn == true) {
      //set brake on the arm 
      brakeSolenoidLow.set(Value.kForward);
    }
    else {
      brakeSolenoidLow.set(Value.kReverse);
    }
  }
  */

  //return radius
  public double getBottomPosition() {
    return m_bottomEncoder.getPosition();
  }

  public double getAbsoluteBottom() {
    //getAbsolutePosition() return in [0,1]
   return Math.toRadians(m_bottomDutyCycleEncoder.getAbsolutePosition() * -360 + m_bottomArmOffset.get());
  }

  public void updateFromAbsoluteBottom() {
    //todo: check REV system error
    ErrorCheck.errREV(m_bottomEncoder.setPosition(getAbsoluteBottom()));
  }

  public boolean areEncodersSynced() {
    System.out.println("*****areEncodersSynced*****");
    boolean syncResult;
      //set sparkmax encoder position
    updateFromAbsoluteBottom();
    System.out.println("getAbsoluteBottom " + getAbsoluteBottom());
    System.out.println("getBottomPosition " + getBottomPosition());
    syncResult = Math.abs(getAbsoluteBottom() - getBottomPosition()) < ArmConfig.ENCODER_SYNCING_TOLERANCE;
    System.out.println("******SyncIteration****** " + "******Result******"  + syncResult);
    return syncResult; 
  }
  public void stopMotors() {
    m_bottomArm.stopMotor();
  }
  public void burnFlash() {
    ErrorCheck.errREV(m_bottomArm.burnFlash());
  }
}
