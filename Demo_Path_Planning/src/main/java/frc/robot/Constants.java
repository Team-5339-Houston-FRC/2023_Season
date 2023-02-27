// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

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
    public static final int kPneumaticHubCANID = 1;
    public static final int kLeftLeadMotorCANID = 12;
    public static final int kRightLeadMotorCANID = 10;
    public static final int kLeftFollowMotorCANID = 13;
    public static final int kRightFollowMotorCANID = 11;
    public static final int kRetractMotorCANID = 21;
    public static final int kPivotMotorCANID = 20;

    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;
    public static final boolean kPivotMotorEncoderReversed = false;
    public static final boolean kRetractEncoderReversed = false;

    public static final double kTrackwidthMeters = 0.64679;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackwidthMeters);

    public static final int kEncoderCPR = 1; // Set to 1 because Spark Max
    public static final double kWheelDiameterMeters = 0.1524; // 6 inches
    public static final double kGearRatio = 10.71; // 10.71:1 gearing
    public static final double kDPPOffset = 0;
    //public static final double kEncoderDistancePerPulse = (kWheelDiameterMeters * Math.PI)
    //    / (double) (kGearRatio / kEncoderCPR) + kDPPOffset;
    public static final double kEncoderDistancePerPulse = 0.0445;

    public static final double ksVolts = 0.23583;
    public static final double kvVoltSecondsPerMeter = 2.6772;
    public static final double kaVoltSecondsSquaredPerMeter = 0.41051;

    public static final double kPDriveVel = 2.9677;
  }

  public static final class RobotArmConstants {
    public static final int kMinPressure = 100;
    public static final int kMaxPressure = 120;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double kOffBalanceAngleThresholdDegrees = 1;
    public static final double kOonBalanceAngleThresholdDegrees = 1;
  }
}
