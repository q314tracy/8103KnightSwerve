// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // width of chassis track width
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // length of chassis wheelbase
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // kinematics
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // maximum speeds achievable
    public static final double kMaxSpeedMetersPerSecond = ModuleConstants.kDriveWheelFreeSpeedRps * ModuleConstants.kWheelCircumferenceMeters;
    public static final double kMaxAngularSpeed = (2 * kMaxSpeedMetersPerSecond) / kTrackWidth; // radians per second

    // Angular offsets of the modules relative to the chassis in radians
    // USE THIS TO SAVE ENCODER OFFSETS
    public static final double kFrontLeftChassisAngularOffset = 0;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = 0;
    public static final double kBackRightChassisAngularOffset = 0;

    // can IDs
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 13;
    public static final int kFrontRightDrivingCanId = 15;
    public static final int kRearRightDrivingCanId = 17;

    public static final int kFrontLeftTurningCanId = 10;
    public static final int kRearLeftTurningCanId = 12;
    public static final int kFrontRightTurningCanId = 14;
    public static final int kRearRightTurningCanId = 16;

    public static final int kFrontLeftTurningEncCanID = 18;
    public static final int kRearLeftTurningEncCanID = 19;
    public static final int kFrontRightTurningEncCanID = 20;
    public static final int kRearRightTurningEncCanID = 21;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {

    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    public static final double kDrivingMotorReduction = 5.36; //L3 for mk4
    public static final double kDriveWheelFreeSpeedRps = kDrivingMotorFreeSpeedRps / kDrivingMotorReduction;

    public static final double kTurningRatio = 18.75; //mk4 azimuth ratio
    public static final double kTurningMotorFreeSpeedRPS = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kTurningMaxSpeedRads = (kTurningMotorFreeSpeedRPS / kTurningRatio) * 2 * Math.PI;
    public static final double kTurningkS = 0; //minimum voltage required to induce movement
    public static final double kTurningkVtrim = 0.95; //use to trim the kV to dial in characterization better
    public static final double kTurninkkV = (12 / kTurningMaxSpeedRads) * kTurningkVtrim;
    public static final double kTurningkP = 0.3; // bump up for more response with large disturbances
    public static final double kTurningkI = 0; // do not use
    public static final double kTurningkD = 0; // use only if steady state oscillation occurs
    public static final TrapezoidProfile.Constraints kTurningConstraints = new TrapezoidProfile.Constraints(
        kTurningMaxSpeedRads,
        kTurningMaxSpeedRads * 8 // really high = really fast accel
    );
    public static final boolean kTurningEncoderInverted = false;
  }

  // constants for operator control
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.2;
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAngularSpeed = 2 * Math.PI;
  }

  // constants for autonomous control
  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  // convenience
  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
