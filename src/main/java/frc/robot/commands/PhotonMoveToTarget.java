// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.naming.spi.DirStateFactory.Result;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PhotonMoveToTarget extends CommandBase {
    double CAMERA_HEIGHT_METERS = 0.3;
    double TARGET_HEIGHT_METERS = 0.4;
    double CAMERA_PITCH_RADIANS = 0;
  /** Creates a new ExampleCommand. */
  PhotonCamera camera1;
  public PhotonMoveToTarget() {
    camera1 = new PhotonCamera("OV9281");
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var result = camera1.getLatestResult();
    if (result.hasTargets()){
        double range =
                        PhotonUtils.calculateDistanceToTargetMeters(
                                CAMERA_HEIGHT_METERS,
                                TARGET_HEIGHT_METERS,
                                CAMERA_PITCH_RADIANS,
                                Units.degreesToRadians(result.getBestTarget().getPitch()));
        double yaw = result.getBestTarget().getYaw();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
