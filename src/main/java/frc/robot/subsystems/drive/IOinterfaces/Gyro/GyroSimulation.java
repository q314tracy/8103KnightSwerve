// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.IOinterfaces.Gyro;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import frc.robot.utils.Constants;

/** Add your docs here. */
public class GyroSimulation implements GyroIO {

    private final ADXRS450_Gyro m_gyro;
    private final ADXRS450_GyroSim m_gyrosim;

    public GyroSimulation() {
        m_gyro = new ADXRS450_Gyro();
        m_gyrosim = new ADXRS450_GyroSim(m_gyro);
    }

    @Override
    public void updatePosition(double pos) {
        m_gyrosim.setAngle(Units.radiansToDegrees(pos) + m_gyro.getAngle());
    }

    @Override
    public void updateRate(double rate) {
        m_gyrosim.setRate(Units.radiansToDegrees(rate));
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
