// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.IOinterfaces.SparkMax;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface SparkMaxIO extends Subsystem {
  
  public void setVoltage(double volts);
  public RelativeEncoder getEncoder();
  public SparkClosedLoopController getPIDController();
  public void periodic();
}
