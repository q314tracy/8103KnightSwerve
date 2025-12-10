// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.commands;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Swerve;


public class RotateChassis extends SequentialCommandGroup {

  private final Swerve m_drive;

  private final SwerveModuleState[] zero_states = new SwerveModuleState[] {
    new SwerveModuleState(),
    new SwerveModuleState(),
    new SwerveModuleState(),
    new SwerveModuleState()
  };

  public RotateChassis(Swerve swerve_drive) {

    //injected subsystem
    m_drive = swerve_drive;

    //command composer
    addCommands(
      Commands.run(() -> m_drive.setModuleStates(zero_states)).withTimeout(2),
      Commands.runEnd(() -> m_drive.drive(0, 0, 0.5, false), () -> m_drive.stop()).withTimeout(5),
      Commands.run(() -> m_drive.setModuleStates(zero_states)).withTimeout(2)
    );

    //be sure to require subsystem
    addRequirements(m_drive);
  }
}
