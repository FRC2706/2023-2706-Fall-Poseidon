// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotcontainers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
// import frc.robot.commands.ArmPneumaticsCommands.*;
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

  /* Create Subsystems in a specific order */

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public PoseidonContainer() {
    // Setup default commands

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link CommandXboxController} or other ways.
   */
  private void configureButtonBindings() {
    /* Driver Controls */
    /*
    ArmPneumaticsSubsystem commands
    // X - Remove Top Brake
    driver.x().onTrue(new RemoveTopBrake());
    // Y - Add Top Brake
    driver.y().onTrue(new AddTopBrake());

    // A - Remove Bottom Brake
    driver.a().onTrue(new RemoveBottomBrake());
    // B - Add Bottom Brake
    driver.b().onTrue(new AddBottomBrake());
    */
    // X - Turn off both gripper solenoids electronically
    driver.x().onTrue(GripperSubsystem.getInstance().stopCommand());
    // Y - No Pressure
    driver.y().onTrue(GripperSubsystem.getInstance().noPressureCommand());
    // A - Low Pressure
    driver.a().onTrue(GripperSubsystem.getInstance().lowPressureCommand());
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
