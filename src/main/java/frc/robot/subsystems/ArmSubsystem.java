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
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.Config;
import frc.robot.ErrorCheck;
import frc.robot.ProfiledPIDFFController;
import frc.robot.SubsystemChecker;
import frc.robot.SubsystemChecker.SubsystemType;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  private static ArmSubsystem instance = null; //static object that contains all movement controls

  private static final MotorType motorType = MotorType.kBrushless; //defines brushless motortype
  public final CANSparkMax m_bottomArm; //bottom SparkMax motor controller
  public final CANSparkMax m_topArm; //bottom SparkMax motor controller

  
  //network table entry
  private final String m_tuningTableBottom = "Arm/BottomArmTuning";
  private final String m_dataTableBottom = "Arm/BottomArmData";
  private final String m_tuningTableTop = "Arm/TopArmTuning";
  private final String m_dataTableTop = "Arm/TopArmData";
  
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
  private DoublePublisher m_bottomTargetAngle;
  private DoublePublisher m_bottomArmPosPub;

  //network tables entries for top arm 
  private DoubleEntry m_topArmPSubs;
  private DoubleEntry m_topArmISubs;
  private DoubleEntry m_topArmDSubs;
  private DoubleEntry m_topArmIzSubs;
  private DoubleEntry m_topArmFFSubs;
  private DoublePublisher m_topArmSetpointPub;   
  private DoublePublisher m_topArmVelPub;
  private DoubleEntry m_topArmMomentToVoltage;
  private DoublePublisher m_topArmFFTestingVolts;
  private DoubleEntry m_topArmOffset;
  private DoublePublisher m_topTargetAngle;
  private DoublePublisher m_topArmPosPub;

  //for bottom arm ff
  private DoubleSubscriber momentToVoltageConversion;
  private double m_bottomVoltageConversion;
  //for top arm ff 
  private double m_topVoltageConversion;


  //spark max absolute encoder
  SparkMaxAbsoluteEncoder m_bottomAbsEncoder;
  SparkMaxAbsoluteEncoder m_topAbsEncoder;

  //embedded relative encoder
  //private RelativeEncoder m_bottomEncoder;
  public SparkMaxPIDController m_pidControllerBottomArm;  
  public SparkMaxPIDController m_pidControllerTopArm;

  ProfiledPIDFFController m_profiledFFController = new ProfiledPIDFFController();


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

    m_topArm = new CANSparkMax(Config.CANID.TOP_ARM_SPARK_CAN_ID, motorType);
    ErrorCheck.errREV(m_topArm.restoreFactoryDefaults());
    ErrorCheck.errREV(m_topArm.setSmartCurrentLimit(ArmConfig.CURRENT_LIMIT));
    m_topArm.setInverted(ArmConfig.TOP_SET_INVERTED);
    ErrorCheck.errREV(m_topArm.setIdleMode(IdleMode.kBrake));

    ErrorCheck.errREV(m_topArm.setSoftLimit(SoftLimitDirection.kForward, ArmConfig.top_arm_forward_limit));
    ErrorCheck.errREV(m_topArm.setSoftLimit(SoftLimitDirection.kReverse, ArmConfig.top_arm_reverse_limit));
    ErrorCheck.errREV(m_topArm.enableSoftLimit(SoftLimitDirection.kForward, ArmConfig.TOP_SOFT_LIMIT_ENABLE));
    ErrorCheck.errREV(m_topArm.enableSoftLimit(SoftLimitDirection.kReverse, ArmConfig.TOP_SOFT_LIMIT_ENABLE));

    m_topArm.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    m_topArm.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);

    m_bottomArm.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    m_bottomArm.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);


    //unit: radius
    m_bottomAbsEncoder = m_bottomArm.getAbsoluteEncoder(Type.kDutyCycle);
    m_bottomAbsEncoder.setInverted(true);
    m_bottomAbsEncoder.setPositionConversionFactor(2*Math.PI);
    m_bottomAbsEncoder.setVelocityConversionFactor(2*Math.PI/60.0);
    m_bottomAbsEncoder.setZeroOffset(Math.toRadians(52));

    m_topAbsEncoder = m_topArm.getAbsoluteEncoder(Type.kDutyCycle);
    m_topAbsEncoder.setInverted(false);
    m_topAbsEncoder.setPositionConversionFactor(2*Math.PI);
    m_topAbsEncoder.setVelocityConversionFactor(2*Math.PI/60.0);
    m_topAbsEncoder.setZeroOffset(Math.toRadians(286));

    m_pidControllerBottomArm = m_bottomArm.getPIDController();
    m_pidControllerBottomArm.setFeedbackDevice(m_bottomAbsEncoder);

    m_pidControllerTopArm = m_topArm.getPIDController();
    m_pidControllerTopArm.setFeedbackDevice(m_topAbsEncoder);

    NetworkTable topArmTuningTable = NetworkTableInstance.getDefault().getTable(m_tuningTableTop);
    m_topArmPSubs = topArmTuningTable.getDoubleTopic("P").getEntry(ArmConfig.top_arm_kP, PubSubOption.periodic(0.02));
    m_topArmISubs = topArmTuningTable.getDoubleTopic("I").getEntry(ArmConfig.top_arm_kI);
    m_topArmDSubs = topArmTuningTable.getDoubleTopic("D").getEntry(ArmConfig.top_arm_kD);
    m_topArmIzSubs = topArmTuningTable.getDoubleTopic("IZone").getEntry(ArmConfig.top_arm_kIz);
    m_topArmFFSubs = topArmTuningTable.getDoubleTopic("FF").getEntry(ArmConfig.top_arm_kFF);
   //m_topArmOffset = topArmTuningTable.getDoubleTopic("Offset").getEntry(ArmConfig.top_arm_offset);
    momentToVoltageConversion = topArmTuningTable.getDoubleTopic("VoltageConversion").subscribe(m_topVoltageConversion);

    NetworkTable bottomArmTuningTable = NetworkTableInstance.getDefault().getTable(m_tuningTableBottom);
    m_bottomArmPSubs = bottomArmTuningTable.getDoubleTopic("P").getEntry(ArmConfig.bottom_arm_kP);
    m_bottomArmISubs = bottomArmTuningTable.getDoubleTopic("I").getEntry(ArmConfig.bottom_arm_kI);
    m_bottomArmDSubs = bottomArmTuningTable.getDoubleTopic("D").getEntry(ArmConfig.bottom_arm_kD);
    m_bottomArmIzSubs = bottomArmTuningTable.getDoubleTopic("IZone").getEntry(ArmConfig.bottom_arm_kIz);
    m_bottomArmFFSubs = bottomArmTuningTable.getDoubleTopic("FF").getEntry(ArmConfig.bottom_arm_kFF);
    //m_bottomArmOffset = bottomArmTuningTable.getDoubleTopic("Offset").getEntry(ArmConfig.bottom_arm_offset);
    momentToVoltageConversion = bottomArmTuningTable.getDoubleTopic("VoltageConversion").subscribe(m_bottomVoltageConversion);

    m_bottomArmFFSubs.accept(ArmConfig.bottom_arm_kFF);
    m_bottomArmPSubs.accept(ArmConfig.bottom_arm_kP);
    m_bottomArmISubs.accept(ArmConfig.bottom_arm_kI);
    m_bottomArmDSubs.accept(ArmConfig.bottom_arm_kD);
    m_bottomArmIzSubs.accept(ArmConfig.bottom_arm_kIz);
    //m_bottomArmOffset.accept(ArmConfig.bottom_arm_offset);

    m_topArmFFSubs.accept(ArmConfig.top_arm_kFF);
    m_topArmPSubs.accept(ArmConfig.top_arm_kP);
    m_topArmISubs.accept(ArmConfig.top_arm_kI);
    m_topArmDSubs.accept(ArmConfig.top_arm_kD);
    m_topArmIzSubs.accept(ArmConfig.top_arm_kIz);

    //m_bottomArmOffset = bottomArmTuningTable.getDoubleTopic("Offset").getEntry(ArmConfig.bottom_arm_offset);
    //m_bottomArmOffset.accept(ArmConfig.bottom_arm_offset);

    NetworkTable bottomArmDataTable = NetworkTableInstance.getDefault().getTable(m_dataTableBottom);
    NetworkTable topArmDataTable = NetworkTableInstance.getDefault().getTable(m_dataTableTop);

    
    m_bottomArmPosPub = bottomArmDataTable.getDoubleTopic("MeasuredAngle").publish(PubSubOption.periodic(0.02));
    m_bottomTargetAngle = bottomArmDataTable.getDoubleTopic("TargetAngle").publish(PubSubOption.periodic(0.02));
    m_bottomArmVelPub = bottomArmDataTable.getDoubleTopic("Vel").publish(PubSubOption.periodic(0.02));

    m_topArmPosPub = topArmDataTable.getDoubleTopic("MeasuredAngle").publish(PubSubOption.periodic(0.02));
    m_topTargetAngle = topArmDataTable.getDoubleTopic("TargetAngle").publish(PubSubOption.periodic(0.02));
    m_topArmVelPub = topArmDataTable.getDoubleTopic("Vel").publish(PubSubOption.periodic(0.02));


    ErrorCheck.errREV(m_pidControllerBottomArm.setFF(m_bottomArmFFSubs.get()));
    ErrorCheck.errREV(m_pidControllerBottomArm.setP(m_bottomArmPSubs.get()));
    ErrorCheck.errREV(m_pidControllerBottomArm.setI(m_bottomArmISubs.get()));
    ErrorCheck.errREV(m_pidControllerBottomArm.setD(m_bottomArmDSubs.get()));
    ErrorCheck.errREV(m_pidControllerBottomArm.setIZone(m_bottomArmIzSubs.get())); 
    ErrorCheck.errREV(m_pidControllerBottomArm.setOutputRange(ArmConfig.min_output, ArmConfig.max_output));

    ErrorCheck.errREV(m_pidControllerTopArm.setFF(m_topArmFFSubs.get()));
    ErrorCheck.errREV(m_pidControllerTopArm.setP(m_topArmPSubs.get()));
    ErrorCheck.errREV(m_pidControllerTopArm.setI(m_topArmISubs.get()));
    ErrorCheck.errREV(m_pidControllerTopArm.setD(m_topArmDSubs.get()));
    ErrorCheck.errREV(m_pidControllerTopArm.setIZone(m_topArmIzSubs.get())); 
    ErrorCheck.errREV(m_pidControllerTopArm.setOutputRange(ArmConfig.min_output, ArmConfig.max_output));


    //to do: could be moved to another spot
    updatePIDSettings();
    //updateFromAbsoluteBottom();
  }

  public void updatePIDSettings() {
    ErrorCheck.errREV(m_pidControllerBottomArm.setFF(m_bottomArmFFSubs.get()));
    ErrorCheck.errREV(m_pidControllerBottomArm.setP(m_bottomArmPSubs.get()));
    ErrorCheck.errREV(m_pidControllerBottomArm.setI(m_bottomArmISubs.get()));
    ErrorCheck.errREV(m_pidControllerBottomArm.setD(m_bottomArmDSubs.get()));
    ErrorCheck.errREV(m_pidControllerBottomArm.setIZone(m_bottomArmIzSubs.get()));
    ErrorCheck.errREV(m_pidControllerBottomArm.setOutputRange(ArmConfig.min_output, ArmConfig.max_output));

    ErrorCheck.errREV(m_pidControllerTopArm.setFF(m_topArmFFSubs.get()));
    ErrorCheck.errREV(m_pidControllerTopArm.setP(m_topArmPSubs.get()));
    ErrorCheck.errREV(m_pidControllerTopArm.setI(m_topArmISubs.get()));
    ErrorCheck.errREV(m_pidControllerTopArm.setD(m_topArmDSubs.get()));
    ErrorCheck.errREV(m_pidControllerTopArm.setIZone(m_topArmIzSubs.get()));
    ErrorCheck.errREV(m_pidControllerTopArm.setOutputRange(ArmConfig.min_output, ArmConfig.max_output));

  }
  

  @Override
  public void periodic() {
    m_bottomArmPosPub.accept(Math.toDegrees(m_bottomAbsEncoder.getPosition()));
    m_bottomArmVelPub.accept(m_bottomAbsEncoder.getVelocity());
    m_bottomTargetAngle.accept(Math.toDegrees(getAbsoluteBottom()));

    m_topArmPosPub.accept(Math.toDegrees(m_topAbsEncoder.getPosition()));
    m_topArmVelPub.accept(m_topAbsEncoder.getVelocity());
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

  public void setTopJointAngle(double angle_top) {
    if (angle_top<Math.toRadians(0) || angle_top>Math.toRadians(35)) {
      angle_top = Math.toRadians(10);
    }
    //setReference angle is in radians)
    //todo: tune FF 
    double targetPos = m_profiledFFController.getNextProfiledPIDPos(getTopPosition(), angle_top);
    m_pidControllerTopArm.setReference((targetPos), ControlType.kPosition, 0, calculateFFTop());
    m_topTargetAngle.accept(Math.toDegrees(targetPos));
    System.out.println(targetPos);

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
    return m_bottomAbsEncoder.getPosition();
  }
  public double getTopPosition() {
    return m_topAbsEncoder.getPosition();
  }

  public double getAbsoluteBottom() {
    //getAbsolutePosition() return in [0,1]
   //return Math.toRadians(m_bottomDutyCycleEncoder.getAbsolutePosition() * -360 + m_bottomArmOffset.get());
  //double absPosition = m_bottomAbsEncoder.getPosition();
  //double adjAbsPosition = absPosition-((int) absPosition/360)*360.0;
   //System.out.println("Get abs position(degree) " + absPosition);
   //System.out.println("adjusted abs position " + adjAbsPosition);
   return m_bottomAbsEncoder.getPosition(); //+ m_bottomArmOffset.get());
  }

  public double getAbsoluteTop() {
    return m_topAbsEncoder.getPosition();
  }

  //public void updateFromAbsoluteBottom() { 
    //todo: check REV system error
    //ErrorCheck.errREV(m_bottomEncoder.setPosition(getAbsoluteBottom()));
  //}

  public boolean areEncodersSynced() {
    System.out.println("*****areEncodersSynced*****");
    boolean syncResult;
      //set sparkmax encoder position
    //updateFromAbsoluteBottom();
    System.out.println("getAbsoluteBottom " + getAbsoluteBottom());
    System.out.println("getBottomPosition " + getBottomPosition());
    syncResult = Math.abs(getAbsoluteBottom() - getBottomPosition()) < ArmConfig.ENCODER_SYNCING_TOLERANCE;
    System.out.println("******SyncIteration****** " + "******Result******"  + syncResult);

    System.out.println("getAbsoluteTop " + getAbsoluteTop());
    System.out.println("getTopPosition " + getTopPosition());
    syncResult = Math.abs(getAbsoluteTop() - getTopPosition()) < ArmConfig.ENCODER_SYNCING_TOLERANCE;
    System.out.println("******SyncIteration****** " + "******Result******"  + syncResult);

    return syncResult; 
  }
  public void stopMotors() {
    m_bottomArm.stopMotor();
    m_topArm.stopMotor();
  }
  public void burnFlash() {
    ErrorCheck.errREV(m_bottomArm.burnFlash());
    ErrorCheck.errREV(m_topArm.burnFlash());
  }
  private double calculateFFTop() {
    double enc2AtHorizontal = getTopPosition() - (Math.PI - getBottomPosition());
    double voltsAtHorizontal;
    voltsAtHorizontal = 2.0;
    //System.out.println("top position " + getTopPosition());
    //System.out.println("bottom position " + getBottomPosition());
    //System.out.println("calculated position " + enc2AtHorizontal);
    System.out.println("return voltage " + voltsAtHorizontal * Math.cos(enc2AtHorizontal));
    return voltsAtHorizontal * Math.cos(enc2AtHorizontal);
  }
}
