// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.deprecated;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */
public class DriveSim implements DriveInterface {
  @Override
  public Pose2d getPose() {
    return new Pose2d();
  }
  
  @Override
  public void periodic() {

  }

  @Override
  public void resetOdometry(Pose2d pose) {

  }

  @Override
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

  }

  @Override
  public void setX() {

  }

  @Override
  public void setModuleStates(SwerveModuleState[] desiredStates) {

  }

  @Override
  public void resetEncoders() {

  }

  @Override
  public void zeroHeading() {

  }

  @Override
  public double getHeading() {
    return 0.0;
  }

  @Override
  public double getTurnRate() {
    return 0.0;
  }
}
