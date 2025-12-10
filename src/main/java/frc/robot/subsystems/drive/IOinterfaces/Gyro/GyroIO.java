// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.IOinterfaces.Gyro;

import edu.wpi.first.wpilibj2.command.Subsystem;

/** Add your docs here. */
public interface GyroIO extends Subsystem {

    public void updatePosition(double pos);
    public void updateRate(double rate);
    public double getAngle();
    public void reset();
    
}
