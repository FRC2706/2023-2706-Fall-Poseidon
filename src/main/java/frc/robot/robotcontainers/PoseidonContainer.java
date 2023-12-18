  
  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotcontainers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.commands.ControlSingleArm;
import frc.robot.commands.SetBottomArm;
import frc.robot.commands.SyncArmEncoders;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.ArmPneumaticsCommands.AddBottomBrake;
import frc.robot.commands.ArmPneumaticsCommands.AddTopBrake;
import frc.robot.commands.ArmPneumaticsCommands.RemoveBottomBrake;
import frc.robot.commands.ArmPneumaticsCommands.RemoveTopBrake;
import frc.robot.subsystems.ArmPneumaticsSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

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

  private final SwerveSubsystem s_Swerve = new SwerveSubsystem();
  private final ArmSubsystem s_arm = ArmSubsystem.getInstance();

  /* Create Subsystems in a specific order */

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public PoseidonContainer() {
    // Setup default commands
    // s_Swerve.setDefaultCommand(
    //     new TeleopSwerve(
    //         s_Swerve,
    //         () -> -driver.getRawAxis(translationAxis),
    //         () -> -driver.getRawAxis(strafeAxis),
    //         () -> -driver.getRawAxis(rotationAxis)
    //     )
    // );
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link CommandXboxController} or other ways.
   */
  private void configureButtonBindings() {
    CommandXboxController driver = new CommandXboxController(0);
    CommandXboxController operator = new CommandXboxController (1);
    /* Driver Controls */
    // driver.a().onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    
    // //ArmPneumaticsSubsystem commands
    // // X - Remove Top Brake
    // driver.leftBumper().onTrue(new RemoveTopBrake());
    // // Y - Add Top Brake
    // driver.leftTrigger().onTrue(new AddTopBrake());

    // // A - Remove Bottom Brake
    // driver.rightBumper().onTrue(new RemoveBottomBrake());
    // // B - Add Bottom Brake
    // driver.rightTrigger().onTrue(new AddBottomBrake());

    // // X - Turn off both gripper solenoids electronically
    // driver.x().onTrue(GripperSubsystem.getInstance().stopCommand());
    // // Y - No Pressure
    // driver.y().onTrue(GripperSubsystem.getInstance().noPressureCommand());
    // // A - Low Pressure
    // driver.a().onTrue(GripperSubsystem.getInstance().lowPressureCommand());
    // // B - High Pressure
    // driver.b().onTrue(GripperSubsystem.getInstance().highPressureCommand());


    /* Operator Controls */
    // operator.a().onTrue(new SetBottomArm(90));
    // operator.b().onTrue(new SetBottomArm(80));
    // operator.y().onTrue(new SyncArmEncoders());

    // operator.a().whileTrue(Commands.run(() -> 
    //     ArmSubsystem.getInstance().setAngles(
    //       Math.toRadians(75), 
    //       Math.toRadians(90), 0, 0), 
    //       ArmSubsystem.getInstance()))
    //   .onTrue(Commands.runOnce(ArmSubsystem.getInstance()::resetProfiledPIDControllers))
    //   .onFalse(Commands.runOnce(ArmSubsystem.getInstance()::stopMotors));

    // operator.b().whileTrue(Commands.run(() -> 
    //     ArmSubsystem.getInstance().setAngles(
    //       Math.toRadiglans(45), 
    //       Math.toRadians(180), 0, 0), 
    //       ArmSubsystem.getInstance()))
    //   .onTrue(Commands.runOnce(ArmSubsystem.getInstance()::resetProfiledPIDControllers))
    //   .onFalse(Commands.runOnce(ArmSubsystem.getInstance()::stopMotors));

    // Control the brakes
    operator.leftTrigger().toggleOnTrue(ArmPneumaticsSubsystem.getInstance().getToggleBottomCommand());

    operator.rightTrigger().toggleOnTrue(ArmPneumaticsSubsystem.getInstance().getToggleTopCommand());

    
    operator.a().whileTrue(new ControlSingleArm(true, Math.toRadians(90)));
    operator.b().whileTrue(new ControlSingleArm(true, Math.toRadians(135)));
    operator.y().whileTrue(new ControlSingleArm(true, Math.toRadians(180)));

    // operator.a().whileTrue(new ControlSingleArm(false, Math.toRadians(10)));
    // operator.b().whileTrue(new ControlSingleArm(false, Math.toRadians(30)));
    // operator.y().whileTrue(new ControlSingleArm(false, Math.toRadians(45)));

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

