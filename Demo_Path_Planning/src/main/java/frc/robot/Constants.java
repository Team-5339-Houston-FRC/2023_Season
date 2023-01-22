// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kLeftLeadMotorCANID = 1;
    public static final int kRightLeadMotorCANID = 2;
    public static final int kLeftFollowMotorCANID = 3;
    public static final int kRightFollowMotorCANID = 4;

    //public static final int[] kLeftEncoderPorts = new int[] {0, 1};
    //public static final int[] kRightEncoderPorts = new int[] {2, 3};
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

    // TODO: Get track width
    public static final double kTrackwidthMeters = 0.57589;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final int kEncoderCPR = 1; // Set to 1 because Spark Max
    public static final double kWheelDiameterMeters = 0.1524; // 6 inches
    public static final double kGearRatio = 10.71; // 10.71:1 gearing
    public static final double kEncoderDistancePerPulse =
        (kWheelDiameterMeters * Math.PI) / (double) (kGearRatio / kEncoderCPR);

    public static final double ksVolts = 0.25775;
    public static final double kvVoltSecondsPerMeter = 8.0323;
    public static final double kaVoltSecondsSquaredPerMeter = 1.6576;

    public static final double kPDriveVel = 0.0020817;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double kOffBalanceAngleThresholdDegrees = 1;
    public static final double kOonBalanceAngleThresholdDegrees  = 1;
  }
}
