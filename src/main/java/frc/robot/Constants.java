package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.lib3512.config.SwerveModuleConstants;

public final class Constants {
  public static final class CanID {
    public static final int PIGEON = 6;

    public static final int MOD0_DRIVE = 0;
    public static final int MOD0_STEER = 0;
    public static final int MOD0_CANCODER = 0;
    
    public static final int MOD1_DRIVE = 0;
    public static final int MOD1_STEER = 0;
    public static final int MOD1_CANCODER = 0;
    
    public static final int MOD2_DRIVE = 0;
    public static final int MOD2_STEER = 0;
    public static final int MOD2_CANCODER = 0;
    
    public static final int MOD3_DRIVE = 0;
    public static final int MOD3_STEER = 0;
    public static final int MOD3_CANCODER = 0;
  }

  public static final class Swerve {
    public static final double JOYSTICK_DEADBAND = 0.1;

    /* Drivetrain Constants */
    public static final double TRACK_WIDTH = Units.inchesToMeters(22);
    public static final double WHEEL_BASE = Units.inchesToMeters(26.75);
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static final double MK4_L1_GEAR_RATIO = (50.0/14.0)*(19.0/25.0)*(45.0/15.0);
    public static final double DRIVE_GEAR_RATIO = MK4_L1_GEAR_RATIO; // 8.14:1
    public static final double STEER_GEAR_RATIO = (12.8 / 1.0); // 12.8:1

    public static final SwerveDriveKinematics SWERVE_KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

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
    public static final double DRIVE_POS_CONVERSION =
        (WHEEL_DIAMETER * Math.PI) / DRIVE_GEAR_RATIO;
    public static final double DRIVE_VEL_CONVERSION = DRIVE_POS_CONVERSION / 60.0;
    public static final double STEER_POS_CONVERSION = 360.0 / STEER_GEAR_RATIO;

    /* Swerve max attainable speeds */
    public static final double MAX_ATTAINABLE_SPEED = 3; // meters per second
    // public static final double MAX_ATTAINABLE_ANGULAR_SPEED = Math.PI*3.0; // radians per second

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
      public static final Rotation2d STEER_OFFSET = Rotation2d.fromDegrees(-91.922462);
      public static final SwerveModuleConstants CONSTANTS =
          new SwerveModuleConstants(DRIVE_MOTOR_ID, STEER_MOTOR_ID, CANCODER_ID, STEER_OFFSET);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int DRIVE_MOTOR_ID = CanID.MOD1_DRIVE;
      public static final int STEER_MOTOR_ID = CanID.MOD1_STEER;
      public static final int CANCODER_ID = CanID.MOD1_CANCODER;
      public static final Rotation2d STEER_OFFSET = Rotation2d.fromDegrees(53.050964);
      public static final SwerveModuleConstants CONSTANTS =
          new SwerveModuleConstants(DRIVE_MOTOR_ID, STEER_MOTOR_ID, CANCODER_ID, STEER_OFFSET);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int DRIVE_MOTOR_ID = CanID.MOD2_DRIVE;
      public static final int STEER_MOTOR_ID = CanID.MOD2_STEER;
      public static final int CANCODER_ID = CanID.MOD2_CANCODER;
      public static final Rotation2d STEER_OFFSET = Rotation2d.fromDegrees(9.264709);
      public static final SwerveModuleConstants CONSTANTS =
          new SwerveModuleConstants(DRIVE_MOTOR_ID, STEER_MOTOR_ID, CANCODER_ID, STEER_OFFSET);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int DRIVE_MOTOR_ID = CanID.MOD3_DRIVE;
      public static final int STEER_MOTOR_ID = CanID.MOD3_STEER;
      public static final int CANCODER_ID = CanID.MOD3_CANCODER;
      public static final Rotation2d STEER_OFFSET = Rotation2d.fromDegrees(2.114444);
      public static final SwerveModuleConstants CONSTANTS =
          new SwerveModuleConstants(DRIVE_MOTOR_ID, STEER_MOTOR_ID, CANCODER_ID, STEER_OFFSET);
    }
  }
}
