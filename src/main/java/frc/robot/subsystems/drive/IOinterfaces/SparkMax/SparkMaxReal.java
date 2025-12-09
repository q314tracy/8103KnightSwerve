// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.IOinterfaces.SparkMax;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SparkMaxReal implements SparkMaxIO {

  private final SparkMax m_motor;
  private final RelativeEncoder m_encoder;
  private final SparkClosedLoopController m_pid;

  public SparkMaxReal(int canID, SparkMaxConfig config) {

    m_motor = new SparkMax(canID, MotorType.kBrushless);
    m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_encoder = m_motor.getEncoder();
    m_pid = m_motor.getClosedLoopController();
  }
  
  @Override
  public void setVoltage(double volts) {
    m_motor.setVoltage(volts);
  }

  @Override
  public RelativeEncoder getEncoder() {
    return m_encoder;
  }

  @Override
  public SparkClosedLoopController getPIDController() {
    return m_pid;
  }

  @Override
  public void periodic() {
  }
}
