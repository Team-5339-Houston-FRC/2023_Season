// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  private final CANSparkMax m_leftLeadMotor = 
    new CANSparkMax(DriveConstants.kLeftLeadMotorCANID, MotorType.kBrushless);
  private final CANSparkMax m_leftFollowMotor = 
    new CANSparkMax(DriveConstants.kLeftFollowMotorCANID, MotorType.kBrushless);

  // The motors on the right side of the drive.
  private final CANSparkMax m_rightLeadMotor = 
    new CANSparkMax(DriveConstants.kRightLeadMotorCANID, MotorType.kBrushless);
  private final CANSparkMax m_rightFollowMotor = 
    new CANSparkMax(DriveConstants.kRightFollowMotorCANID, MotorType.kBrushless);

  // The robot's drive
  private DifferentialDrive m_drive;

  // The left-side drive encoder
  private final RelativeEncoder m_leftEncoder = m_leftLeadMotor.getEncoder();

  // The right-side drive encoder
  private final RelativeEncoder m_rightEncoder = m_rightLeadMotor.getEncoder();

  private double m_leftMotorVoltage, m_rightMotorVoltage;

  // The gyro sensor
  private AHRS m_ahrs;

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  private GenericEntry leftEncoderPulseCount, rightEncoderPulseCount, averageEncoderPulseCount,
    yawDegrees, pitchDegrees, rollDegrees, rotation2D, turnRate, leftMotorVoltage, rightMotorVoltage, 
    leftWheelSpeed, rightWheelSpeed, positionRotation, positionX, positionY;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(AHRSSubsystem ahrs) {

    m_ahrs = ahrs.getAHRS();

    m_leftLeadMotor.restoreFactoryDefaults();
    m_leftFollowMotor.restoreFactoryDefaults();
    m_rightLeadMotor.restoreFactoryDefaults();
    m_rightFollowMotor.restoreFactoryDefaults();

    m_leftLeadMotor.setIdleMode(IdleMode.kBrake);
    m_leftFollowMotor.setIdleMode(IdleMode.kBrake);
    m_rightLeadMotor.setIdleMode(IdleMode.kBrake);
    m_rightFollowMotor.setIdleMode(IdleMode.kBrake);

    m_leftFollowMotor.follow(m_leftLeadMotor);
    m_rightFollowMotor.follow(m_rightLeadMotor);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightLeadMotor.setInverted(true);
    m_rightFollowMotor.setInverted(true);

    m_leftLeadMotor.setInverted(true);
    m_leftFollowMotor.setInverted(true);

    // Sets the distance per pulse for the encoders
    //m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    //m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    m_rightEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    m_leftEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setVelocityConversionFactor(DriveConstants.kEncoderDistancePerPulse / 60);
    m_leftEncoder.setVelocityConversionFactor(DriveConstants.kEncoderDistancePerPulse / 60);

    m_drive = new DifferentialDrive(m_leftLeadMotor, m_rightLeadMotor);

    resetEncoders();
    resetYaw();

    m_odometry =
        new DifferentialDriveOdometry(
            m_ahrs.getRotation2d(), this.getLeftEncoderPosition(), this.getRightEncoderPosition());
    m_leftMotorVoltage = 0;
    m_rightMotorVoltage = 0;
    initHMI();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_ahrs.getRotation2d(),this.getLeftEncoderPosition(), this.getRightEncoderPosition());

    
    updateHMI();
  }

  private double getLeftEncoderPosition() {
    return m_leftEncoder.getPosition();
  }

  private double getRightEncoderPosition() {
    return m_leftEncoder.getPosition();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    resetYaw();
    m_odometry.resetPosition(
        m_ahrs.getRotation2d(), this.getLeftEncoderPosition(), this.getRightEncoderPosition(), pose);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
      m_drive.arcadeDrive(fwd, rot);
  }

  public void autoBalance() {
    double pitch = this.getRoll();
    double fwd = 0;
    double rot = 0;

    //System.out.println("test");

    if (Math.abs(pitch) >= Math.abs(AutoConstants.kOffBalanceAngleThresholdDegrees)) {
      double pitchAngleRadians = pitch * (Math.PI / 180.0);
      if (fwd >= 0)
      fwd = Math.sin(pitchAngleRadians) * 1.5;
      //System.out.println("test1");
    }
    else {
      fwd = 0;
      rot = 0;
    }

    

    //System.out.println("test2 fwd:" + fwd + " rot:" + rot);
    m_drive.arcadeDrive(rot, fwd);
    //System.out.println("test3");

  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotorVoltage = leftVolts;
    m_rightMotorVoltage = rightVolts;
    m_leftLeadMotor.setVoltage(m_leftMotorVoltage);
    m_rightLeadMotor.setVoltage(m_rightMotorVoltage);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (this.getLeftEncoderPosition() + this.getRightEncoderPosition()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public RelativeEncoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public RelativeEncoder getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the Yaw of the robot. */
  public void resetYaw() {
    m_ahrs.reset();
  }

  /**
   * Returns the Yaw of the robot.
   *
   * @return the robot's Yaw in degrees, from -180 to 180
   */
  public double getYaw() {
    return m_ahrs.getYaw();
  }

  /**
   * Returns the Pitch of the robot.
   *
   * @return the robot's Pitch in degrees, from -180 to 180
   */
  public double getPitch() {
    return m_ahrs.getPitch();
  }

  /**
   * Returns the Roll of the robot.
   *
   * @return the robot's Roll in degrees, from -180 to 180
   */
  public double getRoll() {
    return m_ahrs.getRoll();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_ahrs.getRate();
  }

  public void initHMI() {
    ShuffleboardTab driveDiagTab = Shuffleboard.getTab("Drive Diag");

    ShuffleboardLayout gyroLayout = driveDiagTab.getLayout("Gyro", "Grid Layout").withPosition(0, 0).withSize(2, 2)
        .withProperties(Map.of("number of columns", 2, "number of rows", 2));
    rotation2D = gyroLayout.add("Rotation 2D", "").withWidget(BuiltInWidgets.kTextView).withPosition(0, 0).getEntry();
    yawDegrees = gyroLayout.add("Yaw", "").withWidget(BuiltInWidgets.kTextView).withPosition(0, 1).getEntry();
    pitchDegrees = gyroLayout.add("Pitch", "").withWidget(BuiltInWidgets.kTextView).withPosition(1, 0).getEntry();
    rollDegrees = gyroLayout.add("Roll", "").withWidget(BuiltInWidgets.kTextView).withPosition(1, 1).getEntry();

    ShuffleboardLayout encoderLayout = driveDiagTab.getLayout("Encoders", "Grid Layout").withPosition(0, 2)
      .withSize(3, 1).withProperties(Map.of("number of columns", 3, "number of rows", 1));
    leftEncoderPulseCount = encoderLayout.add("Left Pulse Count", "").withWidget(BuiltInWidgets.kTextView).withPosition(0, 0).getEntry();
    rightEncoderPulseCount = encoderLayout.add("Right Pulse Count", "").withWidget(BuiltInWidgets.kTextView).withPosition(1, 0).getEntry();
    averageEncoderPulseCount = encoderLayout.add("Average Pulse Count", "").withWidget(BuiltInWidgets.kTextView).withPosition(2, 0).getEntry();
  
    ShuffleboardLayout motorVoltagesLayout = driveDiagTab.getLayout("Motor Voltages", "Grid Layout").withPosition(0, 3)
      .withSize(2, 1).withProperties(Map.of("number of columns", 2, "number of rows", 1));
    leftMotorVoltage = motorVoltagesLayout.add("Left Voltage", "").withWidget(BuiltInWidgets.kTextView).withPosition(0, 0).getEntry();
    rightMotorVoltage = motorVoltagesLayout.add("Right Voltage", "").withWidget(BuiltInWidgets.kTextView).withPosition(1, 0).getEntry();

    ShuffleboardLayout ratesLayout = driveDiagTab.getLayout("Drive Rates", "Grid Layout").withPosition(2, 0)
      .withSize(3, 1).withProperties(Map.of("number of columns", 3, "number of rows", 1));
    turnRate = ratesLayout.add("Turn Rate", "").withWidget(BuiltInWidgets.kTextView).withPosition(0, 0).getEntry();
    leftWheelSpeed = ratesLayout.add("Left Wheel Speed", "").withWidget(BuiltInWidgets.kTextView).withPosition(1, 0).getEntry();
    rightWheelSpeed = ratesLayout.add("Right Wheel Speed", "").withWidget(BuiltInWidgets.kTextView).withPosition(2, 0).getEntry();

    ShuffleboardLayout positionLayout = driveDiagTab.getLayout("Position", "Grid Layout").withPosition(2, 1)
      .withSize(3, 1).withProperties(Map.of("number of columns", 3, "number of rows", 1));
    positionRotation = positionLayout.add("Rotation", "").withWidget(BuiltInWidgets.kTextView).withPosition(0, 0).getEntry();
    positionX = positionLayout.add("X", "").withWidget(BuiltInWidgets.kTextView).withPosition(1, 0).getEntry();
    positionY = positionLayout.add("Y", "").withWidget(BuiltInWidgets.kTextView).withPosition(2, 0).getEntry();

  }

  public void updateHMI() {

    rotation2D.setString(this.m_ahrs.getRotation2d().getDegrees() + " deg" );
    yawDegrees.setString(this.getYaw() + " deg");
    pitchDegrees.setString(this.getPitch() + " deg");
    rollDegrees.setString(this.getRoll() + " deg");

    leftEncoderPulseCount.setString(this.getLeftEncoderPosition() + " p");
    rightEncoderPulseCount.setString(this.getRightEncoderPosition() + " p");
    averageEncoderPulseCount.setString(this.getAverageEncoderDistance() + " p");

    leftMotorVoltage.setString(this.m_leftMotorVoltage + " V");
    rightMotorVoltage.setString(this.m_leftMotorVoltage + " V");

    turnRate.setString(this.getTurnRate() + " deg//s");
    leftWheelSpeed.setString(this.getWheelSpeeds().leftMetersPerSecond + " m//s");
    rightWheelSpeed.setString(this.getWheelSpeeds().rightMetersPerSecond + " m//s");

    positionRotation.setString(this.getPose().getRotation().getDegrees() + " deg");
    positionX.setString(this.getPose().getX() + " m");
    positionY.setString(this.getPose().getY() + " m");

    
  }
}
