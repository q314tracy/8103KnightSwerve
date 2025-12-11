// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.drive.commands.DriveModules;
import frc.robot.subsystems.drive.commands.RotateChassis;
import frc.robot.subsystems.drive.commands.RotateModules;
import frc.robot.utils.Telemetry;
import frc.robot.utils.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems
  private final Swerve m_robotDrive;

  // telemetry subclass, no calls needed, runs in periodic
  private final Telemetry m_telemetry;

  // The driver's controller
  private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  // chooser for autonomous
  private final SendableChooser<Command> m_autochooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands. Duh.
   */
  public RobotContainer() {

    //start datalogger
    DataLogManager.start();

    // declare subsystems
    m_robotDrive = new Swerve();
    m_telemetry = new Telemetry(m_robotDrive);
    
    // Configure the button bindings
    configureButtonBindings();
    
    // Configure default commands
    m_robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand(
          () -> m_robotDrive.drive(
              -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
              true),
          m_robotDrive));
  

    // build autochooser and post widget
    // IDrun subroutines are also included here, can comment out later to make less cluttered
    m_autochooser.setDefaultOption("no auto", Commands.print("schedule an auto, ya goober"));
    m_autochooser.addOption("IDrun_RotateChassis", new RotateChassis(m_robotDrive));
    m_autochooser.addOption("IDrun_DriveModules", new DriveModules(m_robotDrive));
    m_autochooser.addOption("IDrun_RotateModules", new RotateModules(m_robotDrive));
    SmartDashboard.putData(m_autochooser);
  }

  /**
   * Put your button bindings here, yo.
   */
  private void configureButtonBindings() {
  }

  /**
   * Gets called by Robot.java to schedule autonomous command. Return the command or the autochooser result here.
   */
  public Command getAutonomousCommand() {
    return m_autochooser.getSelected();
  }
}
