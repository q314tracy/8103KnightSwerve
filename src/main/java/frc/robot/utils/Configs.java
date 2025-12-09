package frc.robot.utils;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.utils.Constants.ModuleConstants;

public final class Configs {
  public static final class MAXSwerveModule {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
      // Use module constants to calculate conversion factors and feed forward gain.
      double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
          / ModuleConstants.kDrivingMotorReduction;
      // double turningFactor = 2 * Math.PI;
      double drivingVelocityFeedForward = 12.6 / (5676 * drivingFactor);
      double turningFactor = 2 * Math.PI;

      drivingConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50);
      drivingConfig.encoder
          .positionConversionFactor(drivingFactor) // meters
          .velocityConversionFactor(drivingFactor / 60.0); // meters per second
      drivingConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // adjust the gains appropriately
          .pid(0.05, 0, 0)
          .velocityFF(drivingVelocityFeedForward)
          .outputRange(-1, 1);

      turningConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(40);
      // only used during simulation to update cancoder
      turningConfig.encoder
          .positionConversionFactor(turningFactor) // rads
          .velocityConversionFactor(turningFactor / 60); // rads per second
    }
  }
}
