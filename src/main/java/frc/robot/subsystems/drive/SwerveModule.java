// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;

import com.revrobotics.RelativeEncoder;

import frc.robot.subsystems.drive.IOinterfaces.CANcoder.CANcoderIO;
import frc.robot.subsystems.drive.IOinterfaces.CANcoder.CANcoderReal;
import frc.robot.subsystems.drive.IOinterfaces.CANcoder.CANcoderSimulation;
import frc.robot.subsystems.drive.IOinterfaces.SparkMax.SparkMaxIO;
import frc.robot.subsystems.drive.IOinterfaces.SparkMax.SparkMaxReal;
import frc.robot.subsystems.drive.IOinterfaces.SparkMax.SparkMaxSimulation;
import frc.robot.utils.Configs;
import frc.robot.utils.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase{
  private final SparkMaxIO m_drivingSpark;
  private final SparkMaxIO m_turningSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final CANcoderIO m_turningEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SimpleMotorFeedforward m_turnMotorFeedforward;
  private final ProfiledPIDController m_turningClosedLoopController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());







  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public SwerveModule(int drivingCANId, int turningCANId, int turningEncCANId, double chassisAngularOffset) {

    if (RobotBase.isSimulation()) {
      m_drivingSpark = new SparkMaxSimulation(drivingCANId, Configs.MAXSwerveModule.drivingConfig);
      m_turningSpark = new SparkMaxSimulation(turningCANId, Configs.MAXSwerveModule.turningConfig);
      m_turningEncoder = new CANcoderSimulation(turningEncCANId);
    } else {
      m_drivingSpark = new SparkMaxReal(drivingCANId, Configs.MAXSwerveModule.drivingConfig);
      m_turningSpark = new SparkMaxReal(turningCANId, Configs.MAXSwerveModule.turningConfig);
      m_turningEncoder = new CANcoderReal(turningEncCANId);
    }

    m_drivingEncoder = m_drivingSpark.getEncoder();

    m_drivingClosedLoopController = m_drivingSpark.getPIDController();
    m_turnMotorFeedforward = new SimpleMotorFeedforward(
        ModuleConstants.kTurningkS,
        ModuleConstants.kTurninkkV);
    m_turningClosedLoopController = new ProfiledPIDController(
        ModuleConstants.kTurningkP,
        ModuleConstants.kTurningkI,
        ModuleConstants.kTurningkD,
        ModuleConstants.kTurningConstraints);
    m_turningClosedLoopController.enableContinuousInput(-Math.PI, Math.PI);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }





  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }







  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }






  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {

    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS towards their respective setpoints.
    m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    
    // calculate turning PID and FF, write to motor
    m_turningClosedLoopController.setGoal(correctedDesiredState.angle.getRadians());
    double turningFF = m_turnMotorFeedforward.calculate(m_turningClosedLoopController.getSetpoint().velocity);
    double turningPID = m_turningClosedLoopController
        .calculate(m_turningEncoder.getPosition());
    m_turningSpark.setVoltage(turningFF + turningPID);

    // set desired state
    m_desiredState = desiredState;
  }






  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }






  //functions used during identification runs of drive subsystem

  /** Used during identification run to check for turn motor and encoder inversions. */
  public void rotate() {
    m_turningSpark.setVoltage(0.5);
  }

  /** Used during identification run to check for drive motor inversions. */
  public void drive() {
    m_drivingSpark.setVoltage(0.5);
  }

  /** Used during identification run to stop motors. */
  public void stop() {
    m_turningSpark.setVoltage(0);
    m_drivingSpark.setVoltage(0);
  }





  @Override
  public void periodic() {

    //check if simulation, if yes, update simulation data
    if (RobotBase.isSimulation()) {
      m_drivingSpark.periodic();
      m_turningSpark.periodic();
      m_turningEncoder.updatePosition(m_turningSpark.getEncoder().getVelocity());
    }
  }
}