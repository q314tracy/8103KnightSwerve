// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.commands;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Swerve;

public class DriveModules extends SequentialCommandGroup {

  private final Swerve m_drive;

  public DriveModules(Swerve swerve_drive) {

    //injected subsystem
    m_drive = swerve_drive;

    //command composer
    addCommands(Commands.run(
      () -> m_drive.setModuleStates(new SwerveModuleState[] {
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState()
      })).withTimeout(2),
      Commands.runEnd(() -> m_drive.driveModules(), () -> m_drive.stop()).withTimeout(5));

      //be sure to require subsystem
      addRequirements(m_drive);
  }
}
