  
  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotcontainers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.commands.PhotonMoveToTarget;
import frc.robot.commands.PhotonWaitForData;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.ArmPneumaticsCommands.*;
import frc.robot.subsystems.GripperSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic
 * should actually be handled in the {@link Robot} periodic methods
 * (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class PoseidonContainer extends RobotContainer {
  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final SwerveSubsystem s_Swerve = SwerveSubsystem.getInstance();

  /* Create Subsystems in a specific order */

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public PoseidonContainer() {
    // Setup default commands
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis)
        )
    );
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link CommandXboxController} or other ways.
   */
  private void configureButtonBindings() {
    /* Driver Controls */
    driver.start().onTrue(SwerveSubsystem.getInstance().setHeadingCommand(new Rotation2d(0)));
    driver.back().whileTrue(SwerveSubsystem.getInstance().lockWheelsInX());

    //ArmPneumaticsSubsystem commands
    // X - Remove Top Brake
    driver.leftBumper().onTrue(new RemoveTopBrake());
    // Y - Add Top Brake
    driver.leftTrigger().onTrue(new AddTopBrake());

    // A - Remove Bottom Brake
    driver.rightBumper().onTrue(new RemoveBottomBrake());
    // B - Add Bottom Brake
    driver.rightTrigger().onTrue(new AddBottomBrake());

    // X - Turn off both gripper solenoids electronically
    driver.x().onTrue(GripperSubsystem.getInstance().stopCommand());
    // Y - No Pressure
    driver.y().onTrue(GripperSubsystem.getInstance().noPressureCommand());
    // A - Low Pressure
    //driver.a().onTrue(GripperSubsystem.getInstance().lowPressureCommand());
    // A - Align to apriltag
    driver.a().whileTrue(Commands.sequence(
      new PhotonWaitForData(-1), new ScheduleCommand(Commands.sequence
            (new PhotonMoveToTarget(new Translation2d(-1.5,-0.3), 0.2),
            new PhotonMoveToTarget(new Translation2d(-1,0.05),Rotation2d.fromDegrees(0), 0.01)))))
      .onFalse(new InstantCommand( () -> {},SwerveSubsystem.getInstance()));

    // B - High Pressure
    driver.b().onTrue(GripperSubsystem.getInstance().highPressureCommand());
    /* Operator Controls */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
