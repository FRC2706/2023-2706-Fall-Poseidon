package frc.robot;

import java.io.BufferedReader;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.lib3512.config.SwerveModuleConstants;

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
   * ID 4: MiniSwerve (Small swerve chassis)
   * ID 5: NeoBeetle (Small Neo tank drive)
   * ID 6: ArmBot (Arm Bot)
   * 
   * ...
   * 
   * 
   **/

  public static final class CanID {
    public static final int PIGEON = robotSpecific(30, 27, 27, 27, 30);

    public static final int MOD0_DRIVE = 24;
    public static final int MOD0_STEER = 23;
    public static final int MOD0_CANCODER = 9;

    public static final int MOD1_DRIVE = 21;
    public static final int MOD1_STEER = 25;
    public static final int MOD1_CANCODER = 6;

    public static final int MOD2_DRIVE = 20;
    public static final int MOD2_STEER = 26;
    public static final int MOD2_CANCODER = 8;

    public static final int MOD3_DRIVE = 27;
    public static final int MOD3_STEER = 22;
    public static final int MOD3_CANCODER = 7;
  }

  public static final class Swerve {
    public static final double JOYSTICK_DEADBAND = 0.1;

    /* Drivetrain Constants */
    public static final double TRACK_WIDTH = robotSpecific(0.52);
    public static final double WHEEL_BASE = robotSpecific(0.655);
    public static final double WHEEL_DIAMETER = robotSpecific(0.1016);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static final double MK4_L1_GEAR_RATIO = (50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0);
    public static final double DRIVE_GEAR_RATIO = MK4_L1_GEAR_RATIO; // 8.14:1
    public static final double STEER_GEAR_RATIO = (12.8 / 1.0); // 12.8:1

    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0), // FL
        new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0), // FR
        new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0), // RL
        new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)); // RR

    /* Swerve Voltage Compensation */
    public static final double VOLTAGE_COMPENSATION = 12.0;

    /* Swerve Current Limiting */
    public static final int STEER_CURRENT_LIMIT = 20;
    public static final int DRIVE_CURRENT_LIMIT = 80;

    /* Steer Motor PID Values */
    public static final double STEER_P = 0.01;
    public static final double STEER_I = 0.0;
    public static final double STEER_D = 0.0;
    public static final double STEER_FF = 0.0;

    /* Drive Motor PID Values */
    public static final double DRIVE_P = 0.1;
    public static final double DRIVE_I = 0.0;
    public static final double DRIVE_D = 0.0;
    public static final double DRIVE_FF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double DRIVE_S = 0.667;
    public static final double DRIVE_V = 2.44;
    public static final double DRIVE_A = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double DRIVE_POS_CONVERSION = (WHEEL_DIAMETER * Math.PI) / DRIVE_GEAR_RATIO;
    public static final double DRIVE_VEL_CONVERSION = DRIVE_POS_CONVERSION / 60.0;
    public static final double STEER_POS_CONVERSION = 360.0 / STEER_GEAR_RATIO;

    /* Swerve max attainable speeds */
    public static final double MAX_ATTAINABLE_SPEED = 3; // meters per second
    // public static final double MAX_ATTAINABLE_ANGULAR_SPEED = Math.PI*3.0; //
    // radians per second

    /* Swerve Teleop speeds */
    public static final double TELEOP_SPEED = 1.0; // meters per second
    public static final double TELEOP_ANGULAR_SPEED = Math.PI; // radians per second

    /* Neutral Modes */
    public static final IdleMode STEER_IDLE_MODE = IdleMode.kBrake;
    public static final IdleMode DRIVE_IDLE_MODE = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean DRIVE_INVERT = false;
    public static final boolean STEER_INVERT = false;

    /* Steer Encoder Invert */
    public static final boolean CANCODER_INVERT = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int DRIVE_MOTOR_ID = CanID.MOD0_DRIVE;
      public static final int STEER_MOTOR_ID = CanID.MOD0_STEER;
      public static final int CANCODER_ID = CanID.MOD0_CANCODER;
      public static final Rotation2d STEER_OFFSET = Rotation2d.fromDegrees(270.73);
      public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID, STEER_MOTOR_ID,
          CANCODER_ID, STEER_OFFSET);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int DRIVE_MOTOR_ID = CanID.MOD1_DRIVE;
      public static final int STEER_MOTOR_ID = CanID.MOD1_STEER;
      public static final int CANCODER_ID = CanID.MOD1_CANCODER;
      public static final Rotation2d STEER_OFFSET = Rotation2d.fromDegrees(159.3);
      public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID, STEER_MOTOR_ID,
          CANCODER_ID, STEER_OFFSET);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int DRIVE_MOTOR_ID = CanID.MOD2_DRIVE;
      public static final int STEER_MOTOR_ID = CanID.MOD2_STEER;
      public static final int CANCODER_ID = CanID.MOD2_CANCODER;
      public static final Rotation2d STEER_OFFSET = Rotation2d.fromDegrees(194.9);
      public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID, STEER_MOTOR_ID,
          CANCODER_ID, STEER_OFFSET);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int DRIVE_MOTOR_ID = CanID.MOD3_DRIVE;
      public static final int STEER_MOTOR_ID = CanID.MOD3_STEER;
      public static final int CANCODER_ID = CanID.MOD3_CANCODER;
      public static final Rotation2d STEER_OFFSET = Rotation2d.fromDegrees(8.5);
      public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID, STEER_MOTOR_ID,
          CANCODER_ID, STEER_OFFSET);
    }
  }
}
