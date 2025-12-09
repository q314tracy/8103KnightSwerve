// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Swerve;

public class Telemetry extends SubsystemBase {

  private final Swerve m_drive;

  private final Field2d m_field2d;

  //swerve module state publisher
  private final StructArrayPublisher<SwerveModuleState> m_swerveStatePublisher = 
    NetworkTableInstance.getDefault()
        .getStructArrayTopic("SwerveStates", SwerveModuleState.struct) // <-- Use .struct here
        .publish();

  public Telemetry(Swerve swerve_drive) {
    m_drive = swerve_drive;
    m_field2d = new Field2d();
    SmartDashboard.putData(m_field2d);
  }

  @Override
  public void periodic() {

    //drive subsystem telemetry
    SmartDashboard.putNumber("heading", m_drive.getHeading());
    m_swerveStatePublisher.accept(m_drive.getStates());

    m_field2d.setRobotPose(m_drive.getPose());
    
  }
}
