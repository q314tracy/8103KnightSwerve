// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.deprecated;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface DriveInterface extends Subsystem {

  //abstract methods
  public Pose2d getPose();
  public void resetOdometry(Pose2d pose);
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative);
  public void setX();
  public void setModuleStates(SwerveModuleState[] desiredStates);
  public void resetEncoders();
  public void zeroHeading();
  public double getHeading();
  public double getTurnRate();
}