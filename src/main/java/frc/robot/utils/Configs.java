package frc.robot.utils;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.ModuleConstants;

public final class Configs {
  public static final class MAXSwerveModule {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
      // Use module constants to calculate conversion factors and feed forward gain.
      double drivingFactor = ModuleConstants.kWheelCircumferenceMeters / ModuleConstants.kDrivingMotorReduction;
      double turningFactor = 2 * Math.PI;

      // this is totally arbitrary and tuned manually for the current gear ratio.
      // adjustment will be required if ratio is changed in Constants.
      double drivingVelocityFeedForward = 1 / DriveConstants.kMaxSpeedMetersPerSecond;

      drivingConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50)
          .inverted(false);
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
          .smartCurrentLimit(40)
          .inverted(false);
      // only used during simulation to update cancoder
      turningConfig.encoder
          .positionConversionFactor(turningFactor) // rads
          .velocityConversionFactor(turningFactor / 60); // rads per second
    }
  }
}
