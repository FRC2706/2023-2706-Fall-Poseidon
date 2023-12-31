// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.lib.lib2706.ErrorCheck.errSpark;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.lib2706.ProfiledExternalPIDController;
import frc.lib.lib2706.SubsystemChecker;
import frc.lib.lib2706.SubsystemChecker.SubsystemType;
import frc.lib.lib2706.UpdateSimpleFeedforward;
import frc.robot.Config;
import frc.robot.Config.CANID;
import frc.robot.subsystems.ArmConfig.ArmFeedforward;
import frc.robot.subsystems.ArmConfig.ArmSimulation;

public class ArmSubsystem extends SubsystemBase {
  private static ArmSubsystem instance = null;

  private final CANSparkMax m_botSpark, m_topSpark;
  private final SparkMaxPIDController m_botSparkPid, m_topSparkPid;
  private final SparkMaxAbsoluteEncoder m_botEncoder, m_topEncoder;
  
  private final ProfiledExternalPIDController m_botProfiledPid, m_topProfiledPid;

  private final PIDController m_botSimPid, m_topSimPid;
  private boolean m_runBotPidSim = false;
  private boolean m_runTopPidSim = false;

  private final ArmDisplay m_armDisplay;

  private SimpleMotorFeedforward m_botFF;
  private SimpleMotorFeedforward m_topFF;
  private UpdateSimpleFeedforward m_botUpdateFF;
  private UpdateSimpleFeedforward m_topUpdateFF;

  private double lastBotSpeed = 0;
  private double lastTopSpeed = 0;

  private double botFFVoltage = 0;
  private double topFFVoltage = 0;

  private DoublePublisher pubBotPos, pubBotVel, pubTopPos, pubTopVel;
  private DoublePublisher pubBotPosSet, pubTopPosSet;
  private DoublePublisher pubBotPosProSet, pubBotVelProSet, pubTopPosProSet, pubTopVelProSet;
  private DoublePublisher pubBotAccelSet, pubTopAccelSet;
  
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
  private ArmSubsystem() {
    /**
     * Setup CANSparkMaxs
     */
    m_botSpark = new CANSparkMax(CANID.BOTTOM_ARM_SPARK_CAN_ID, Config.NEO_MOTORTYPE);
    m_topSpark = new CANSparkMax(CANID.TOP_ARM_SPARK_CAN_ID, Config.NEO_MOTORTYPE);

    errSpark(() -> m_botSpark.restoreFactoryDefaults());
    errSpark(() -> m_topSpark.restoreFactoryDefaults());

    errSpark(() -> m_botSpark.setSmartCurrentLimit(ArmConfig.BOT_CURRENT_LIMIT));
    errSpark(() -> m_topSpark.setSmartCurrentLimit(ArmConfig.TOP_CURRENT_LIMIT));

    m_botSpark.setInverted(ArmConfig.BOT_SET_INVERTED);
    m_topSpark.setInverted(ArmConfig.TOP_SET_INVERTED);

    errSpark(() -> m_botSpark.setIdleMode(IdleMode.kBrake));
    errSpark(() -> m_topSpark.setIdleMode(IdleMode.kBrake));
    
    errSpark(() -> m_botSpark.enableVoltageCompensation(ArmConfig.BOT_VOLTAGE_COMP));
    errSpark(() -> m_topSpark.enableVoltageCompensation(ArmConfig.TOP_VOLTAGE_COMP));

    /**
     * Setup SparkMaxPidControllers
     */
    m_botSparkPid = m_botSpark.getPIDController();
    m_topSparkPid = m_topSpark.getPIDController();

    errSpark(() -> m_botSparkPid.setP(ArmConfig.BOT_KP));
    errSpark(() -> m_botSparkPid.setI(ArmConfig.BOT_KI));
    errSpark(() -> m_botSparkPid.setD(ArmConfig.BOT_KD));
    errSpark(() -> m_botSparkPid.setIZone(ArmConfig.BOT_IZONE));

    errSpark(() -> m_topSparkPid.setP(ArmConfig.BOT_KP));
    errSpark(() -> m_topSparkPid.setI(ArmConfig.BOT_KI));
    errSpark(() -> m_topSparkPid.setD(ArmConfig.BOT_KD));
    errSpark(() -> m_topSparkPid.setIZone(ArmConfig.BOT_IZONE));

    /**
     * Setup Absolute Encoders
     */
    m_botEncoder = m_botSpark.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    errSpark(() -> m_botEncoder.setPositionConversionFactor(ArmConfig.BOT_POS_CONV_FACTOR));
    errSpark(() -> m_botEncoder.setVelocityConversionFactor(ArmConfig.BOT_VEL_CONV_FACTOR));
    errSpark(() -> m_botEncoder.setInverted(ArmConfig.BOT_ENC_INVERT));
    errSpark(() -> m_botEncoder.setZeroOffset(ArmConfig.BOT_ENC_OFFSET));

    m_topEncoder = m_topSpark.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    errSpark(() -> m_topEncoder.setPositionConversionFactor(ArmConfig.TOP_POS_CONV_FACTOR));
    errSpark(() -> m_topEncoder.setVelocityConversionFactor(ArmConfig.TOP_VEL_CONV_FACTOR));
    errSpark(() -> m_topEncoder.setInverted(ArmConfig.TOP_ENC_INVERT));
    errSpark(() -> m_topEncoder.setZeroOffset(ArmConfig.TOP_ENC_OFFSET));

    // Set status frame to get absolute encoder position every 20 ms
    errSpark(() -> m_botSpark.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20));
    errSpark(() -> m_topSpark.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20));

    // Set status frame to get absolute encoder velocity every 20 ms
    errSpark(() -> m_botSpark.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20));
    errSpark(() -> m_topSpark.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20));

    errSpark(() -> m_botSparkPid.setFeedbackDevice(m_botEncoder));
    errSpark(() -> m_topSparkPid.setFeedbackDevice(m_topEncoder));

    /**
     * Setup soft limits after encoder
     */
    errSpark(() -> m_botSpark.setSoftLimit(SoftLimitDirection.kForward, ArmConfig.BOT_FORW_LIMIT));
    errSpark(() -> m_botSpark.setSoftLimit(SoftLimitDirection.kReverse, ArmConfig.BOT_REV_LIMIT));
    errSpark(() -> m_botSpark.enableSoftLimit(SoftLimitDirection.kForward, ArmConfig.BOT_SOFT_LIMIT_ENABLE));
    errSpark(() -> m_botSpark.enableSoftLimit(SoftLimitDirection.kReverse, ArmConfig.BOT_SOFT_LIMIT_ENABLE));

    errSpark(() -> m_topSpark.setSoftLimit(SoftLimitDirection.kForward, ArmConfig.TOP_FORW_LIMIT));
    errSpark(() -> m_topSpark.setSoftLimit(SoftLimitDirection.kReverse, ArmConfig.TOP_REV_LIMIT));
    errSpark(() -> m_topSpark.enableSoftLimit(SoftLimitDirection.kForward, ArmConfig.TOP_SOFT_LIMIT_ENABLE));
    errSpark(() -> m_topSpark.enableSoftLimit(SoftLimitDirection.kReverse, ArmConfig.TOP_SOFT_LIMIT_ENABLE));

    /**
     * Create the ArmDisplay
     */
    m_armDisplay = new ArmDisplay(
        Units.metersToInches(ArmConfig.BOT_LENGTH), 
        Units.metersToInches(ArmConfig.TOP_LENGTH));

    /**
     * Setup ProfiledPidControllers and Simulation PIDControllers
     */
    m_botProfiledPid = new ProfiledExternalPIDController(
        new Constraints(ArmConfig.BOT_MAX_VEL, ArmConfig.BOT_MAX_ACCEL));

    m_topProfiledPid = new ProfiledExternalPIDController(
        new Constraints(ArmConfig.TOP_MAX_VEL, ArmConfig.TOP_MAX_ACCEL));

    m_botSimPid = new PIDController(ArmConfig.BOT_KP, ArmConfig.BOT_KI, ArmConfig.BOT_KD);
    m_topSimPid = new PIDController(ArmConfig.TOP_KP, ArmConfig.TOP_KI, ArmConfig.TOP_KD);
    
    m_botSimPid.setIZone(ArmConfig.BOT_IZONE);
    m_topSimPid.setIZone(ArmConfig.TOP_IZONE);

    /**
     * Setup networktables
     */
    NetworkTable botTable = NetworkTableInstance.getDefault().getTable(ArmConfig.BOT_DATA_TABLE);
    NetworkTable topTable = NetworkTableInstance.getDefault().getTable(ArmConfig.TOP_DATA_TABLE);

    pubBotPos = botTable.getDoubleTopic("Pos Deg").publish(PubSubOption.periodic(0.02));
    pubBotVel = botTable.getDoubleTopic("Vel DegPerSec").publish(PubSubOption.periodic(0.02));

    pubTopPos = topTable.getDoubleTopic("Pos Deg").publish(PubSubOption.periodic(0.02));
    pubTopVel = topTable.getDoubleTopic("Vel DegPerSec").publish(PubSubOption.periodic(0.02));

    pubBotPosSet = botTable.getDoubleTopic("Pos Setpoint Deg").publish(PubSubOption.periodic(0.02));
    pubBotPosProSet = botTable.getDoubleTopic("Pos ProfileSetpoint Deg").publish(PubSubOption.periodic(0.02));
    pubBotVelProSet = botTable.getDoubleTopic("Vel ProfileSetpoint DegPerSec").publish(PubSubOption.periodic(0.02));
    pubBotAccelSet = botTable.getDoubleTopic("Accel ProfileSetpoint DegPerSecPerSec").publish(PubSubOption.periodic(0.02));
    
    pubTopPosSet = topTable.getDoubleTopic("Pos Setpoint Deg").publish(PubSubOption.periodic(0.02));
    pubTopPosProSet = topTable.getDoubleTopic("Pos ProfileSetpoint Deg").publish(PubSubOption.periodic(0.02));
    pubTopVelProSet = topTable.getDoubleTopic("Vel ProfuleSetpoint DegPerSec").publish(PubSubOption.periodic(0.02));
    pubTopAccelSet = topTable.getDoubleTopic("Accel ProfileSetpoint DegPerSecPerSec").publish(PubSubOption.periodic(0.02));
    
    /**
     * Setup SimpleMotorFeedforward
     */
    m_botFF = ArmConfig.BOT_SIMPLE_FF;
    m_topFF = ArmConfig.TOP_SIMPLE_FF;

    m_botUpdateFF = new UpdateSimpleFeedforward(
      (ff) -> m_botFF = ff, 
      botTable, 
      ArmConfig.BOT_SIMPLE_FF.ks, 
      ArmConfig.BOT_SIMPLE_FF.kv, 
      ArmConfig.BOT_SIMPLE_FF.ka);

    m_topUpdateFF = new UpdateSimpleFeedforward(
      (ff) -> m_topFF = ff, 
      topTable, 
      ArmConfig.TOP_SIMPLE_FF.ks, 
      ArmConfig.TOP_SIMPLE_FF.kv, 
      ArmConfig.TOP_SIMPLE_FF.ka);
    

    /**
     * Burn flash on CANSparkMaxs
     */
    burnFlash();
  }

  public boolean isArmPickupReady() {
    return false;
  }

  @Override
  public void periodic() {
    m_botUpdateFF.checkForUpdates();
    m_topUpdateFF.checkForUpdates();

    m_armDisplay.updateMeasurementDisplay(
        getBotPosition(), 
        getTopPosition());

    pubBotPos.accept(Math.toDegrees(getBotPosition()));
    pubBotVel.accept(Math.toDegrees(getBotVel()));
    pubTopPos.accept(Math.toDegrees(getTopPosition()));
    pubTopVel.accept(Math.toDegrees(getTopVel()));
  }

  /**
   * Set the angles in radians.
   * 
   * @param botAngle The bot angle in radians
   * @param topAngle The top angle in radians
   */
  public void setAngles(double botAngle, double topAngle, double botVel, double topVel) {
    setBotAngle(botAngle, botVel);
    setTopAngle(topAngle, topVel);

    m_armDisplay.updateSetpointDisplay(botAngle, topAngle);
    pubBotPosSet.accept(Math.toDegrees(botAngle));
    pubTopPosSet.accept(Math.toDegrees(topAngle));
  }

  /**
   * Set the bottom angle and velocity.
   * 
   * @param botAngle Angle in rad
   * @param botVel Velocity in rad/s
   */
  public void setBotAngle(double botAngle, double botVel) {
    double pidSetpoint = m_botProfiledPid.calculatePIDSetpoint(
        getBotPosition(), 
        new TrapezoidProfile.State(
            botAngle, 
            botVel));

    double acceleration = (m_botProfiledPid.getSetpoint().velocity - lastBotSpeed) / 0.02;

    botFFVoltage = m_botFF.calculate(m_botProfiledPid.getSetpoint().velocity, acceleration) 
          + calculateGravFFBot(false);

    m_botSparkPid.setReference(pidSetpoint, ControlType.kPosition, 0, botFFVoltage, ArbFFUnits.kVoltage);

    lastBotSpeed = m_botProfiledPid.getSetpoint().velocity;

    pubBotPosProSet.accept(Math.toDegrees(pidSetpoint));
    pubBotVelProSet.accept(Math.toDegrees(m_botProfiledPid.getSetpoint().velocity));
    pubBotAccelSet.accept(Math.toDegrees(acceleration));

    m_runBotPidSim = true;
  }

  /**
   * Set the top angle and velocity.
   * 
   * @param topAngle Angle in rad
   * @param topVel Velocity in rad/s
   */
  public void setTopAngle(double topAngle, double topVel) {
    double pidSetpoint = m_topProfiledPid.calculatePIDSetpoint(
        getTopPosition(), 
        new TrapezoidProfile.State(
            topAngle, 
            topVel));
    
    double acceleration = (m_topProfiledPid.getSetpoint().velocity - lastTopSpeed) / 0.02;


    // If deccelerating and velocity is high, allow more deccelerating.
    // if (acceleration * m_topProfiledPid.getSetpoint().velocity <= 0 && 
    //     Math.abs(m_topProfiledPid.getSetpoint().velocity) > ArmConfig.BOT_MAX_VEL*0.5) {
    //   acceleration *= 1.5;
    // }

    topFFVoltage = m_topFF.calculate(m_topProfiledPid.getSetpoint().velocity, acceleration)
                + calculateGravFFTop(false);

    m_topSparkPid.setReference(pidSetpoint, ControlType.kPosition, 0, topFFVoltage, ArbFFUnits.kVoltage);

    lastTopSpeed = m_topProfiledPid.getSetpoint().velocity;

    pubTopPosProSet.accept(Math.toDegrees(pidSetpoint));
    pubTopVelProSet.accept(Math.toDegrees(m_topProfiledPid.getSetpoint().velocity));
    pubTopAccelSet.accept(Math.toDegrees(acceleration));

    m_runTopPidSim = true;
  }


  /**
   * Get the bottom position in radians.
   * 
   * @return Position in radians.
   */
  public double getBotPosition() {
    if (!RobotBase.isSimulation()) {
      return m_botEncoder.getPosition() - Math.toRadians(10);
    } else {
      return ArmSimulation.ARM_BOT_SIM.getAngleRads();
    }
  }

  /**
   * Get the bottom velocity in radians per second.
   * 
   * @return Velocity in rad/s
   */
  public double getBotVel() {
    if (!RobotBase.isSimulation()) {
      return m_botEncoder.getVelocity();
    } else {
      return ArmSimulation.ARM_BOT_SIM.getVelocityRadPerSec();
    }
  }

  /**
   * Get the top position in radians.
   * 
   * @return Position in radians.
   */
  public double getTopPosition() {
    if (!RobotBase.isSimulation()) {
      return m_topEncoder.getPosition();
    } else {
      return ArmSimulation.ARM_TOP_SIM.getAngleRads();
    }
  }

  /**
   * Get the top velocity in radians per second.
   * 
   * @return Velocity in rad/s
   */
  public double getTopVel() {
    if (!RobotBase.isSimulation()) {
      return m_topEncoder.getVelocity();
    } else {
      return ArmSimulation.ARM_TOP_SIM.getVelocityRadPerSec();
    }
  }

  /**
   * Reset ProfiledPidControllers for current position and velocity.
   * 
   * Must be called if the pid controllers haven't been used in awhile.
   * (Aka call in initalize of commands)
   */
  public void resetProfiledPIDControllers() {
    m_botProfiledPid.reset(getBotPosition(), getBotVel());
    m_topProfiledPid.reset(getTopPosition(), getTopVel());

    botFFVoltage = 0;
    topFFVoltage = 0;

    if (RobotBase.isSimulation()) {
      m_botSimPid.reset();
      m_topSimPid.reset();
    }

    lastBotSpeed = getBotVel();
    lastTopSpeed = getTopVel();
  }

  /**
   * Check if the arm has reached the last given setpoint.
   * 
   * @return True if at setpoint, false for not yet there.
   */
  public boolean isAtSetpoint() {
    return m_botProfiledPid.atGoal(ArmConfig.POSITION_TOLERANCE, ArmConfig.VELOCITY_TOLERANCE)
        && m_topProfiledPid.atGoal(ArmConfig.POSITION_TOLERANCE, ArmConfig.VELOCITY_TOLERANCE);
  }

  private double calculateGravFFBot(boolean haveCone) {
    return calculateGravFFBot(haveCone, getBotPosition(), getTopPosition());
  }

  private double calculateGravFFBot(boolean haveCone, double botAngle, double topAngle) {
    double topAtHorizontal = topAngle - (Math.PI - botAngle);
    double bottomArmMoment = ArmFeedforward.BOT_FORCE * (ArmFeedforward.LENGTH_BOT_TO_COG*Math.cos(botAngle));
    double topArmMoment = ArmFeedforward.TOP_FORCE * (ArmConfig.BOT_LENGTH*Math.cos(botAngle) + ArmFeedforward.LENGTH_TOP_TO_COG*Math.cos(topAtHorizontal));

    if (haveCone == false) {
      return (bottomArmMoment + topArmMoment) * ArmFeedforward.BOT_MOMENT_TO_VOLTAGE;
    } else {
      double coneMoment = ArmFeedforward.CONE_FORCE * (ArmConfig.BOT_LENGTH*Math.cos(botAngle) + ArmConfig.TOP_LENGTH*Math.cos(topAtHorizontal));
      return (bottomArmMoment + topArmMoment + coneMoment) * ArmFeedforward.BOT_MOMENT_TO_VOLTAGE;
    }
  }

  private double calculateGravFFTop(boolean haveCone) {
    return calculateGravFFTop(haveCone, getBotPosition(), getTopPosition());
  }


  private double calculateGravFFTop(boolean haveCone, double botAngle, double topAngle) {
    double topAtHorizontal = topAngle - (Math.PI - botAngle);
    double voltsAtHorizontal = haveCone ? ArmFeedforward.TOP_HORIZONTAL_VOLTAGE_CONE : ArmFeedforward.TOP_HORIZONTAL_VOLTAGE_NOCONE;
    return voltsAtHorizontal * Math.cos(topAtHorizontal);
  }

  /**
   * Stop the motors from moving. Interrupt any previous instructions.
   */
  public void stopMotors() {
    m_botSpark.stopMotor();
    m_topSpark.stopMotor();

    botFFVoltage = 0;
    topFFVoltage = 0;

    m_runBotPidSim = false;
    m_runTopPidSim = false;
  }

  /**
   * Burn flash after a 0.2 second delay to ensure settings are saved.
   */
  private void burnFlash() {
    try
    {
      Thread.sleep(200);
    } catch (Exception e) {}

    errSpark(() -> m_botSpark.burnFlash());
    errSpark(() -> m_topSpark.burnFlash());
  }

  /**
   * Simulation periodic only runs in simulation in vscode and fakes battery draw, motor power and voltage.
   */
  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();

    double botVoltageApplied = 0;
    double topVoltageApplied = 0;

    if (m_runBotPidSim && DriverStation.isEnabled()) {
      // Use the simPid to simulate the CANSparkMax pid controller
      botVoltageApplied = botFFVoltage + 
            m_botSimPid.calculate(getBotPosition(), m_botProfiledPid.getSetpoint().position);
    }

    if (m_runTopPidSim && DriverStation.isEnabled()) {
      // Use the simPid to simulate the CANSparkMax pid controller
      topVoltageApplied = topFFVoltage + 
            m_topSimPid.calculate(getTopPosition(), m_topProfiledPid.getSetpoint().position);
    }

    // Pass the simulation with the inputs as voltages
    ArmSimulation.ARM_BOT_SIM.setInput(botVoltageApplied - calculateGravFFBot(false));
    ArmSimulation.ARM_TOP_SIM.setInput(topVoltageApplied - calculateGravFFTop(false));

    // Update simulation
    ArmSimulation.ARM_BOT_SIM.update(0.020); // 20ms clock cycle
    ArmSimulation.ARM_TOP_SIM.update(0.020);

    // Update encoder readings (doesn't work for AbsoluteEncoder from CANSparkMax)
    // m_botEncoder.setPosition(ArmSimulation.ARM_BOT_SIM.getAngleRads());
    // m_topEncoder.setPosition(ArmSimulation.ARM_TOP_SIM.getAngleRads());

    // Update the simulation of the load on the battery
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
        ArmSimulation.ARM_BOT_SIM.getCurrentDrawAmps(),
        ArmSimulation.ARM_TOP_SIM.getCurrentDrawAmps()));
  }

  public void testFeedForwardTop(double additionalVoltage) {
    double voltage = additionalVoltage + calculateGravFFTop(false);
    m_topSparkPid.setReference(voltage, ControlType.kVoltage);
    System.out.println("Voltage set: " + voltage);
  }

  public void testFeedForwardBot(double additionalVoltage) {
    double voltage = additionalVoltage + calculateGravFFBot(false);
    m_botSparkPid.setReference(voltage, ControlType.kVoltage);
    System.out.println("Voltage set: " + voltage);
  }
}
