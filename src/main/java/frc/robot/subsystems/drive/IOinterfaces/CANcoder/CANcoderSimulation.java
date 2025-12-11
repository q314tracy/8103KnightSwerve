// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.IOinterfaces.CANcoder;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;

import frc.robot.utils.Constants.ModuleConstants;

public class CANcoderSimulation implements CANcoderIO {

  private final CANcoder m_encoder;
  private final CANcoderSimState m_encodersim;

  public CANcoderSimulation(int canID) {
    m_encoder = new CANcoder(canID);
    m_encodersim = new CANcoderSimState(m_encoder);
  }

  @Override
  public void updatePosition(double pos) {
    m_encodersim.addPosition((pos / ModuleConstants.kTurningRatio) / (2*Math.PI) / 50);
  }

  @Override
  public double getPosition() {
    if (ModuleConstants.kTurningEncoderInverted) {
      return -m_encoder.getAbsolutePosition().getValue().in(Radians);
    } else {
      return m_encoder.getAbsolutePosition().getValue().in(Radians);
    }
  }
}
