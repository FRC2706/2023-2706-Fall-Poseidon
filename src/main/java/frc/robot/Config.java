package frc.robot;

import java.io.BufferedReader;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;

public final class Config {
  /**
   * Instructions for set up of robot.conf file on robot
   *
   * 0. Connect to the robot to the robot using a usb cable or the wifi network.
   * 1. Using a tool like Git Bash or putty, ssh into admin@roboRIO-2706-FRC.local
   * (ssh admin@roboRIO-2706-FRC.local)
   * a. There is no password on a freshly flashed roboRIO
   * 2. Go up a directory (cd ..)
   * 3. cd into lvuser/ (cd lvuser/)
   * 4. Create a new file called robot.conf (touch robot.conf)
   * 5. Open the file with vi (vi robot.conf)
   * 6. Press i to enter insert mode
   * 7. Add an integer denoting the robot id. If it's the first robot, use 0,
   * second use 1 etc.
   * 8. Press [ESC] followed by typing :wq in order to save and quit
   * 9. To verify this worked type: more robot.conf
   * 10. If it displays the value you entered, it was successful
   * 11. Type exit to safely exit the ssh session
   */

  private static final Path ROBOT_ID_LOC = Paths.get(System.getProperty("user.home"), "robot.conf");

  /**
   * ID of the robot that code is running on
   */
  private static int robotId = -1;

  
  /**
   * Returns one of the values passed based on the robot ID
   *
   * @param first The first value (default value)
   * @param more  Other values that could be selected
   * @param <T>   The type of the value
   * @return The value selected based on the ID of the robot
   */
  @SafeVarargs
  public static <T> T robotSpecific(T first, T... more) {
    if (getRobotId() < 1 || getRobotId() > more.length) {
      return first;
    } else {
      return more[getRobotId() - 1];
    }
  }

  /**
   * Obtain the robot id found in the robot.conf file
   *
   * @return The id of the robot
   */
  public static int getRobotId() {

    if (robotId < 0) {
      // Set to the ID of the 2023 Competition robot if the simulation is running
      if (RobotBase.isSimulation()) {
        robotId = 0;

        // Not simulation so read the file on the roborio for it's robot id.
      } else {
        try (BufferedReader reader = Files.newBufferedReader(ROBOT_ID_LOC)) {
          robotId = Integer.parseInt(reader.readLine());
        } catch (Exception e) {
          robotId = 0; // DEFAULT TO COMP ROBOT IF NO ID IS FOUND
        }
      }
    }

    return robotId;
  }

  /**
   * ROBOT IDs
   * 
   * ID 0: Poseidon (Charged Up)
   * ID 1: Clutch (Rapid React)
   * ID 2: Beetle (Small Talon tank drive)
   * ID 3: Cosmobot (Deep Space)
   * 
   * ...
   * 
   * 
   **/
  
  /**
     * CAN IDs, ports, channels, etc.
     */
    public static class CANID {   
      public static int PIGEON = robotSpecific(30, 27, 27, 27, 30);
  
      public static int CANDLE = robotSpecific(15, 15, -1, 15, 15);
      public static int CTRE_PCM = robotSpecific(1, 1, -1, -1);

      // Swerve Drive
      public static final int FRONT_LEFT_DRIVE = 24;
      public static final int REAR_LEFT_DRIVE = 20;
      public static final int FRONT_RIGHT_DRIVE = 21;
      public static final int REAR_RIGHT_DRIVE = 27;

      public static final int FRONT_LEFT_STEERING = 23;
      public static final int REAR_LEFT_STEERING = 26;
      public static final int FRONT_RIGHT_STEERING = 25;
      public static final int REAR_RIGHT_STEERING = 22;

      public static final int FRONT_LEFT_CANCODER = 9;
      public static final int REAR_LEFT_CANCODER = 8;
      public static final int FRONT_RIGHT_CANCODER = 6;
      public static final int REAR_RIGHT_CANCODER = 7;

      // Arm Subsystem
      public static final int TOP_ARM_SPARK_CAN_ID = robotSpecific(5, 0, 0, 0, 0, 18, 18);
      public static final int BOTTOM_ARM_SPARK_CAN_ID = robotSpecific(4, 0, 0, 0, 0, 19, 19);
      public static final int TOP_CANCODER_CAN_ID = 3;
      public static final int BOTTOM_CANCODER_CAN_ID = 2;

      // Differential Drive CAN IDs
      public static int DIFF_LEADER_LEFT = robotSpecific(-01, 6, 2, 5, -01, 35);
      public static int DIFF_LEADER_RIGHT = robotSpecific(-01, 3, 1, 3, -01, 33);
      public static int DIFF_FOLLOWER_LEFT = robotSpecific(-01, 5, -1, 7, -01, 37);
      public static int DIFF_FOLLOWER_RIGHT = robotSpecific(-01, 2, -1, 9, -01, 39);
  
      // Clutch CAN IDs
      public static int INTAKE = robotSpecific(-01, 8, -1, -1);
      public static int SHOOTER = robotSpecific(-01, 11, 5, -1);
      public static int CLIMBER = robotSpecific(-01, 4, -1, -1);
      public static int INDEXER = robotSpecific(-01, 7, 7, -1);
  }
  public static class Swerve{
    public static final int steeringCurrentLimit = 20;
    public static final int driveCurrentLimit = 50;

    public static final double driveVoltComp = 12.0;
    public static final double steeringVoltComp = 12.0;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1.35;

    public static final double FL_ENCODER_OFFSET = 90.73;
    public static final double FR_ENCODER_OFFSET = 204.3;
    public static final double RL_ENCODER_OFFSET = 168.9;
    public static final double RR_ENCODER_OFFSET = -5.5; 

    public static boolean INVERTED_FRONT_LEFT_DRIVE = robotSpecific(false);
    public static boolean INVERTED_REAR_LEFT_DRIVE =  robotSpecific(false);
    public static boolean INVERTED_FRONT_RIGHT_DRIVE = robotSpecific(false);
    public static boolean INVERTED_REAR_RIGHT_DRIVE = robotSpecific(false);

    public static boolean INVERTED_FRONT_LEFT_STEERING =  robotSpecific(false);
    public static boolean INVERTED_REAR_LEFT_STEERING =  robotSpecific(false);
    public static boolean INVERTED_FRONT_RIGHT_STEERING = robotSpecific(false);
    public static boolean INVERTED_REAR_RIGHT_STEERING =  robotSpecific(false);

    public static double ENCODER_SYNCING_PERIOD = 0.5; // seconds
    public static int ENCODER_SYNCING_TIMEOUT = 40; // seconds

    public static double TIME_FOR_BRAKES_IN_AUTO = 14.9; // seconds

    public static final double MK4_L1_GEAR_RATIO = (50.0/14.0)*(19.0/25.0)*(45.0/15.0);
    public static final double turningEncoderConstant = (2*Math.PI)/12.8;
    public static final double drivePositionConversionFactor = drivetrainWheelDiameter * Math.PI / MK4_L1_GEAR_RATIO;
    public static final double driveVelocityConversionFactor = drivePositionConversionFactor / 60.0;

    public static final IdleMode defaultDriveIdleMode = IdleMode.kBrake;
    public static final IdleMode defaultSteeringIdleMode = IdleMode.kCoast;

    public static final double drive_kIZone = 0.0;
    public static final double drive_kFF = 0.0; // These can also be module specific.
    public static final double drive_kP = 0.2; // Hopefully they won't need to be.
    public static final double drive_kI = 0.0; // Depends on hardware differences.
    public static final double drive_kD = 0.0;
    
    public static DoubleSubscriber sub_drive_kFF = NetworkTableInstance.getDefault().getTable("SwerveChassis/DrivePID").getDoubleTopic("Drive kFF").subscribe(drive_kFF);
    public static DoubleSubscriber sub_drive_kP = NetworkTableInstance.getDefault().getTable("SwerveChassis/DrivePID").getDoubleTopic("Drive kP").subscribe(drive_kP);
    public static DoubleSubscriber sub_drive_kI = NetworkTableInstance.getDefault().getTable("SwerveChassis/DrivePID").getDoubleTopic("Drive kI").subscribe(drive_kI);
    public static DoubleSubscriber sub_drive_kD = NetworkTableInstance.getDefault().getTable("SwerveChassis/DrivePID").getDoubleTopic("Drive kD").subscribe(drive_kD);
    public static DoubleSubscriber sub_drive_kIZone = NetworkTableInstance.getDefault().getTable("SwerveChassis/DrivePID").getDoubleTopic("Drive kIzone").subscribe(drive_kIZone);


    public static final double steering_kFF = 0.0;
    public static final double steering_kP = 1.0;
    public static final double steering_kI = 0.0;
    public static final double steering_kD = 0.1;
    public static final double steering_kIZone = 0.0; //5 degrees

    public static DoubleSubscriber sub_steering_kFF = NetworkTableInstance.getDefault().getTable("SwerveChassis/DrivePID").getDoubleTopic("Steering kFF").subscribe(steering_kFF);
    public static DoubleSubscriber sub_steering_kP = NetworkTableInstance.getDefault().getTable("SwerveChassis/DrivePID").getDoubleTopic("Steering kP").subscribe(steering_kP);
    public static DoubleSubscriber sub_steering_kI = NetworkTableInstance.getDefault().getTable("SwerveChassis/DrivePID").getDoubleTopic("Steering kI").subscribe(steering_kI);
    public static DoubleSubscriber sub_steering_kD = NetworkTableInstance.getDefault().getTable("SwerveChassis/DrivePID").getDoubleTopic("Steering kD").subscribe(steering_kD);
    public static DoubleSubscriber sub_steering_kIZone = NetworkTableInstance.getDefault().getTable("SwerveChassis/DrivePID").getDoubleTopic("Steering kIzone").subscribe(steering_kIZone);

    public static final SwerveDriveKinematics kSwerveDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),   
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Max speeds for desaturating
    public static final double kMaxAttainableWheelSpeed = 3.0; // Used by Auto, do not change.
    
    // ~ Teleop speeds. Speeds should be portional to kMaxAttainableAngularSpeed. Angular speeds should be specified.
       
    // Default Speeds
    public static final double teleopDefaultSpeed = kMaxAttainableWheelSpeed;// Full speed
    public static final double teleopDefaultAngularSpeed = Math.PI*3.0; // Old value: Math.PI*2.0

    // Left Bumper speeds
    public static final double teleopLeftBumperSpeed = kMaxAttainableWheelSpeed / 3.0;// A third of full speed
    public static final double teleopLeftBumperAngularSpeed = Math.PI; // Can be a slower angular speed if wanted

    // Right bumper speeds
    public static final double teleopRightBumperSpeed = 0.3;
    public static final double teleopRightBumperAngularSpeed = 0.3;
    

    // Old PathWeaver speeds
    public static final double kMaxAutoSpeed = 3; // m/s
    public static final double kMaxAutoAcceleration = 3; // m/s/s
    public static final double kMaxAutoAngularSpeed = Math.PI *3; // rad/s
    public static final double kMaxAutoAngularAcceleration = Math.PI * 3; // rad/s/s

    // Acceleration limit for teleop
    public static final double teleopRateLimit = 3;

    public static final double driveKS = 0.667;
    public static final double driveKV = 2.9;
    public static final double driveKA = 0.5;

    public static DoubleSubscriber sub_kA = NetworkTableInstance.getDefault().getTable("SwerveChassis/DrivePID").getDoubleTopic("Drive kA").subscribe(driveKA);
    public static DoubleSubscriber sub_kV = NetworkTableInstance.getDefault().getTable("SwerveChassis/DrivePID").getDoubleTopic("Drive kV").subscribe(driveKV);
    public static DoubleSubscriber sub_kS = NetworkTableInstance.getDefault().getTable("SwerveChassis/DrivePID").getDoubleTopic("Drive kS").subscribe(driveKS);

    
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAutoAngularSpeed, kMaxAutoAngularAcceleration);
}




  /** ADD CONSTANTS BELOW THIS LINE */


  public static Double DRIVER_JOYSTICK_DEADBAND = 0.1; // TODO: Investigate if this can be better tuned
  
  /** Drivetrain Constants */
  public static final double kTrackWidth = robotSpecific(0.655, 1.2267, 0.3136, 0.569, 0.52, 0.51762);
  public static final double kWheelBase = robotSpecific(0.52,-0.0,-0.0,-0.0,-0.0,-0.0);
  public static double drivetrainWheelDiameter = robotSpecific(0.0986536, 0.1524, 0.1016, 0.1524, 0.01524);

  public static DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidth);
  
  /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;

  /* Swerve Current Limiting */
  public static final int angleContinuousCurrentLimit = 20;
  public static final int driveContinuousCurrentLimit = 80;

  /* Angle Motor PID Values */
  //FIND OUT: if any of below is bot specific
  public static final double angleKP = 0.01;
  public static final double angleKI = 0.0;
  public static final double angleKD = 0.0;
  public static final double angleKFF = 0.0;

  /* Drive Motor PID Values */
  public static final double driveKP = 0.1;
  public static final double driveKI = 0.0;
  public static final double driveKD = 0.0;
  public static final double driveKFF = 0.0;

  /* Drive Motor Characterization Values */
  public static final double driveKS = 0.667;
  public static final double driveKV = 2.44;
  public static final double driveKA = 0.27;
  

}

