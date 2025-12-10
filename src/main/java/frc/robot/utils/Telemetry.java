// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Swerve;

public class Telemetry extends SubsystemBase {

  //injected subsystems
  private final Swerve m_drive;

  //field2d object for visualization
  private final Field2d m_field2d;

  //publisher instances for different data streams
  private final StructArrayPublisher<SwerveModuleState> m_swerveStatePublisher = 
    NetworkTableInstance.getDefault()
        .getStructArrayTopic("SwerveModuleStates", SwerveModuleState.struct)
        .publish();
  private final StructPublisher<ChassisSpeeds> m_speedsPublisher = 
    NetworkTableInstance.getDefault()
        .getStructTopic("ChassisSpeeds", ChassisSpeeds.struct)
        .publish();

  /**
   * Used to abstract away telemetry functions. Inject subsystem instances as params and
   * put telemetry in periodic block. Also instantiate any struct publishers here for custom data types or arrays.
   * @param swerve_drive
   */
  public Telemetry(Swerve swerve_drive) {

    //injected subsystems
    m_drive = swerve_drive;
    m_field2d = new Field2d();
    SmartDashboard.putData(m_field2d);
  }

  //put your telemetry here, nerd
  @Override
  public void periodic() {

    //drive subsystem stuff
    SmartDashboard.putNumber("heading", m_drive.getHeading());
    m_swerveStatePublisher.accept(m_drive.getStates());
    m_speedsPublisher.accept(m_drive.getSpeeds());

    //field2d object
    m_field2d.setRobotPose(m_drive.getPose());
  }
}
