// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.IOinterfaces.CANcoder;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.hardware.CANcoder;

public class CANcoderReal implements CANcoderIO {

  private final CANcoder m_encoder;

  public CANcoderReal(int canID) {
    m_encoder = new CANcoder(canID);
  }

  @Override
  public void updatePosition(double pos) {
    //do nothing
  }

  @Override
  public double getPosition() {
    return m_encoder.getAbsolutePosition().getValue().in(Radians);
  }
}
