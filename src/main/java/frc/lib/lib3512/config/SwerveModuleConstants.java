package frc.lib.lib3512.config;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
  public final int driveMotorID;
  public final int steerMotorID;
  public final int cancoderID;
  public final Rotation2d steerOffset;

  /**
   * Swerve Module Constants to be used when creating swerve modules.
   *
   * @param driveMotorID
   * @param steerMotorID
   * @param canCoderID
   * @param steerOffset
   */
  public SwerveModuleConstants(
      int driveMotorID, int steerMotorID, int canCoderID, Rotation2d steerOffset) {
    this.driveMotorID = driveMotorID;
    this.steerMotorID = steerMotorID;
    this.cancoderID = canCoderID;
    this.steerOffset = steerOffset;
  }
}
