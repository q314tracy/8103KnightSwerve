// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.IOinterfaces.Gyro;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import frc.robot.utils.Constants;

/** Add your docs here. */
public class GyroReal implements GyroIO {

    private final ADXRS450_Gyro m_gyro;

    public GyroReal() {
        m_gyro = new ADXRS450_Gyro();
    }

    @Override
    public void updatePosition(double pos) {
    }

    @Override
    public void updateRate(double rate) {
    }

    @Override
    public double getAngle() {
        if (Constants.DriveConstants.kGyroReversed) {
            return Units.degreesToRadians(-m_gyro.getAngle());
        } else {
            return Units.degreesToRadians(m_gyro.getAngle());
        }
    }

    @Override
    public void reset() {
        m_gyro.reset();
    }
}
