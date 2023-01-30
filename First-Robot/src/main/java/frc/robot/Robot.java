// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private CANSparkMax m_leftLeadDrive;
  private CANSparkMax m_leftFollowDrive;
  private CANSparkMax m_rightLeadDrive;
  private CANSparkMax m_rightFollowDrive;
  private DifferentialDrive m_robotDrive;
  private final XboxController m_controller = new XboxController(0);
  private final Timer m_timer = new Timer();
  private AHRS ahrs;

    // Maps for XBox Controller Diag Screen
    private GenericEntry jstick1_xaxis, jstick1_yaxis, jstick2_xaxis, jstick2_yaxis, leftTrigger, rightTrigger,
        xButton, yButton, bButton, aButton, backButton, startButton, leftBumper, rightBumper, jstick1_click,
        jstick2_click, dpadPOV, leftRumbleToggle, leftRumbleValue, rightRumbleToggle, rightRumbleValue;
        
    // Maps for Navigation Diag Screen
    private GenericEntry ahrsConnected, ahrsCalibrating, accelermoterXAxis, accelermoterYAxis, accelermoterZAxis,
        ahrsIsMoving, ahrsIsRotating, ahrsAccelX, ahrsAccelY, gyroYaw, gyroPitch, gyroRoll, velocityX, velocityY,
        leftEncoderPulseCount, leftEncoderVelocity, leftCountsPerRev, rightEncoderPulseCount, 
        rightEncoderVelocity, rightCountsPerRev;

    // Maps for Comms Diag Screen
    private GenericEntry statusCAN_BusOffCount, statusCAN_PercentBusUtilization, statusCAN_ReceiveErrorCount,
        statusCAN_TransmitErrorCount, statusCAN_TXFullCount;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_leftLeadDrive = new CANSparkMax(2, MotorType.kBrushless);
    m_leftFollowDrive = new CANSparkMax(4, MotorType.kBrushless); 
    m_rightLeadDrive = new CANSparkMax(1, MotorType.kBrushless);
    m_rightFollowDrive= new CANSparkMax(3, MotorType.kBrushless);

    m_leftLeadDrive.restoreFactoryDefaults();
    m_leftFollowDrive.restoreFactoryDefaults();
    m_rightLeadDrive.restoreFactoryDefaults();
    m_rightFollowDrive.restoreFactoryDefaults();

    m_leftFollowDrive.follow(m_leftLeadDrive);
    m_rightFollowDrive.follow(m_rightLeadDrive);

    m_robotDrive = new DifferentialDrive(m_leftLeadDrive, m_rightLeadDrive);

    try {
        ahrs = new AHRS(SPI.Port.kMXP); 
    } catch (RuntimeException ex ) {
        DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }

    initXboxControllerHMI();
    initNavigationHMI();
    initCommsHMI();
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {    
    updateXboxControllerHMI();
    updateNavigationHMI();
    updateCommsHMI();
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    updateXboxControllerHMI();
    updateNavigationHMI();
    updateCommsHMI();
    m_robotDrive.arcadeDrive(m_controller.getLeftX() *.7, m_controller.getLeftY() * .7);
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    updateXboxControllerHMI();
    updateNavigationHMI();
    updateCommsHMI();
  }

  private void initXboxControllerHMI() {
    ShuffleboardTab controllerDiagTab = Shuffleboard.getTab("Controller Diag");

    ShuffleboardLayout leftJoystickLayout = controllerDiagTab.getLayout("Left Joystick", "Grid Layout")
        .withPosition(0, 0).withSize(2, 3).withProperties(Map.of("number of columns", 1, "number of rows", 3));
    jstick1_xaxis = leftJoystickLayout.add("JStick 1 - X Axis", 0).withWidget(BuiltInWidgets.kNumberBar)
        .withPosition(0, 0).withProperties(Map.of("min", -1, "max", 1)).getEntry();
    jstick1_yaxis = leftJoystickLayout.add("JStick 1 - Y Axis", 0).withWidget(BuiltInWidgets.kNumberBar)
        .withPosition(0, 1).withProperties(Map.of("min", -1, "max", 1)).getEntry();
    jstick1_click = leftJoystickLayout.add("Click", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(0, 2)
        .withProperties(Map.of("color when true", "Green", "color when false", "Black")).getEntry();

    ShuffleboardLayout rightJoystickLayout = controllerDiagTab.getLayout("Right Joystick", "Grid Layout")
        .withPosition(2, 0).withSize(2, 3).withProperties(Map.of("number of columns", 1, "number of rows", 3));
    jstick2_xaxis = rightJoystickLayout.add("JStick 2 - X Axis", 0).withWidget(BuiltInWidgets.kNumberBar)
        .withPosition(0, 0).withProperties(Map.of("min", -1, "max", 1)).getEntry();
    jstick2_yaxis = rightJoystickLayout.add("JStick 2 - Y Axis", 0).withWidget(BuiltInWidgets.kNumberBar)
        .withPosition(0, 1).withProperties(Map.of("min", -1, "max", 1)).getEntry();
    jstick2_click = rightJoystickLayout.add("Click", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(0, 2)
        .withProperties(Map.of("color when true", "Green", "color when false", "Black")).getEntry();

    leftTrigger = controllerDiagTab.add("Left Trigger", 0).withWidget(BuiltInWidgets.kNumberBar).withPosition(0, 3)
        .withSize(2, 1).withProperties(Map.of("min", 0, "max", 1)).getEntry();

    rightTrigger = controllerDiagTab.add("Right Trigger", 0).withWidget(BuiltInWidgets.kNumberBar).withPosition(2, 3)
        .withSize(2, 1).withProperties(Map.of("min", 0, "max", 1)).getEntry();

    leftBumper = controllerDiagTab.add("Left Bumper", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(4, 3)
        .withSize(1, 1).withProperties(Map.of("color when true", "Green", "color when false", "Black")).getEntry();

    rightBumper = controllerDiagTab.add("Right Bumper", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(5, 3)
        .withSize(1, 1).withProperties(Map.of("color when true", "Green", "color when false", "Black")).getEntry();

    xButton = controllerDiagTab.add("X Button", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(4, 0)
        .withSize(1, 1).withProperties(Map.of("color when true", "Green", "color when false", "Black")).getEntry();

    yButton = controllerDiagTab.add("Y Button", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(5, 0)
        .withSize(1, 1).withProperties(Map.of("color when true", "Green", "color when false", "Black")).getEntry();

    aButton = controllerDiagTab.add("A Button", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(4, 1)
        .withSize(1, 1).withProperties(Map.of("color when true", "Green", "color when false", "Black")).getEntry();

    bButton = controllerDiagTab.add("B Button", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(5, 1)
        .withSize(1, 1).withProperties(Map.of("color when true", "Green", "color when false", "Black")).getEntry();

    backButton = controllerDiagTab.add("Back", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(4, 2)
        .withSize(1, 1).withProperties(Map.of("color when true", "Green", "color when false", "Black")).getEntry();

    startButton = controllerDiagTab.add("Start", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(5, 2)
        .withSize(1, 1).withProperties(Map.of("color when true", "Green", "color when false", "Black")).getEntry();

    dpadPOV = controllerDiagTab.add("DPAD POV", "Not Pressed").withWidget(BuiltInWidgets.kTextView).withPosition(6, 0)
        .getEntry();

    ShuffleboardLayout leftRumbleLayout = controllerDiagTab.getLayout("Left Rumble", "Grid Layout").withPosition(6, 2)
        .withSize(2, 2).withProperties(Map.of("number of columns", 1, "number of rows", 2));
    leftRumbleValue = leftRumbleLayout.add("Intensity", 0).withWidget(BuiltInWidgets.kNumberSlider).withPosition(0, 0)
        .withProperties(Map.of("min", 0, "max", 1)).getEntry();
    leftRumbleToggle = leftRumbleLayout.add("Toggle", false).withWidget(BuiltInWidgets.kToggleButton).withPosition(0, 1)
        .getEntry();

    ShuffleboardLayout rightRumbleLayout = controllerDiagTab.getLayout("Right Rumble", "Grid Layout").withPosition(8, 2)
        .withSize(2, 2).withProperties(Map.of("number of columns", 1, "number of rows", 2));
    rightRumbleValue = rightRumbleLayout.add("Intensity", 0).withWidget(BuiltInWidgets.kNumberSlider).withPosition(0, 0)
        .withProperties(Map.of("min", 0, "max", 1)).getEntry();
    rightRumbleToggle = rightRumbleLayout.add("Toggle", false).withWidget(BuiltInWidgets.kToggleButton)
        .withPosition(0, 1).getEntry();
  }

  private void initNavigationHMI() {
    ShuffleboardTab driveDiagTab = Shuffleboard.getTab("Drive Diag");

    ahrsConnected = driveDiagTab.add("AHRS Connected", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(0, 0)
        .withSize(1, 1).withProperties(Map.of("color when true", "Green", "color when false", "Black")).getEntry();

    ahrsCalibrating = driveDiagTab.add("AHRS Calibrating", false).withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(1, 0).withSize(1, 1)
        .withProperties(Map.of("color when true", "Green", "color when false", "Black")).getEntry();

    ShuffleboardLayout accelerometerLayout = driveDiagTab.getLayout("Accelerometers", "Grid Layout").withPosition(0, 1)
        .withSize(1, 3).withProperties(Map.of("number of columns", 1, "number of rows", 3));
    accelermoterXAxis = accelerometerLayout.add("X Axis", 0).withWidget(BuiltInWidgets.kNumberBar).withPosition(0, 0)
        .withProperties(Map.of("min", -1, "max", 1)).getEntry();
    accelermoterYAxis = accelerometerLayout.add("Y Axis", 0).withWidget(BuiltInWidgets.kNumberBar).withPosition(0, 0)
        .withProperties(Map.of("min", -1, "max", 1)).getEntry();
    accelermoterZAxis = accelerometerLayout.add("Z Axis", 0).withWidget(BuiltInWidgets.kNumberBar).withPosition(0, 0)
        .withProperties(Map.of("min", -1, "max", 1)).getEntry();

    ShuffleboardLayout gyroLayout = driveDiagTab.getLayout("Headings", "Grid Layout").withPosition(1, 1).withSize(1, 3)
        .withProperties(Map.of("number of columns", 1, "number of rows", 3));
    gyroYaw = gyroLayout.add("Yaw", "").withWidget(BuiltInWidgets.kTextView).withPosition(0, 0).getEntry();
    gyroPitch = gyroLayout.add("Pitch", "").withWidget(BuiltInWidgets.kTextView).withPosition(0, 1).getEntry();
    gyroRoll = gyroLayout.add("Roll", "").withWidget(BuiltInWidgets.kTextView).withPosition(0, 2).getEntry();

    ahrsIsMoving = driveDiagTab.add("Is Moving", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(2, 0)
        .withSize(1, 1).withProperties(Map.of("color when true", "Green", "color when false", "Black")).getEntry();

    ahrsIsRotating = driveDiagTab.add("Is Rotating", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(3, 0)
        .withSize(1, 1).withProperties(Map.of("color when true", "Green", "color when false", "Black")).getEntry();

    velocityX = driveDiagTab.add("Velocity X", "").withWidget(BuiltInWidgets.kTextView).withPosition(2, 2).getEntry();
    velocityY = driveDiagTab.add("Velocity Y", "").withWidget(BuiltInWidgets.kTextView).withPosition(3, 2).getEntry();
    
    ahrsAccelX = driveDiagTab.add("World Linear Accel X", "").withWidget(BuiltInWidgets.kTextView).withPosition(2, 1).getEntry();
    ahrsAccelY = driveDiagTab.add("World Linear Accel Y", "").withWidget(BuiltInWidgets.kTextView).withPosition(3, 1).getEntry();

    ShuffleboardLayout leftMotorLayout = driveDiagTab.getLayout("Left", "Grid Layout").withPosition(4, 1)
    .withSize(2, 2).withProperties(Map.of("number of columns", 1, "number of rows", 3));
    leftEncoderPulseCount = leftMotorLayout.add("Encoder Pulse Count", "").withWidget(BuiltInWidgets.kTextView).withPosition(0, 0).getEntry();
    leftEncoderVelocity = leftMotorLayout.add("Encoder Velocity", "").withWidget(BuiltInWidgets.kTextView).withPosition(0, 1).getEntry();
    leftCountsPerRev = leftMotorLayout.add("Counts Per Rev", "").withWidget(BuiltInWidgets.kTextView).withPosition(0, 2).getEntry();

    ShuffleboardLayout rightMotorLayout = driveDiagTab.getLayout("Right", "Grid Layout").withPosition(6, 1)
    .withSize(2, 2).withProperties(Map.of("number of columns", 1, "number of rows", 3));
    rightEncoderPulseCount = rightMotorLayout.add("Encoder Pulse Count", "").withWidget(BuiltInWidgets.kTextView).withPosition(0, 0).getEntry();
    rightEncoderVelocity = rightMotorLayout.add("Encoder Velocity", "").withWidget(BuiltInWidgets.kTextView).withPosition(0, 1).getEntry();
    rightCountsPerRev = rightMotorLayout.add("Counts Per Rev", "").withWidget(BuiltInWidgets.kTextView).withPosition(0, 2).getEntry();
    
  }

  private void initCommsHMI() {
    ShuffleboardTab commsDiagTab = Shuffleboard.getTab("COMMS Diag");

    ShuffleboardLayout statusCANLayout = commsDiagTab.getLayout("CAN Status", "Grid Layout").withPosition(0, 0)
            .withSize(2, 3)
            .withProperties(Map.of("number of columns", 1, "number of rows", 5, "Label position", "LEFT"));
    statusCAN_BusOffCount = statusCANLayout.add("Off Cnt (S)", 0).withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 0).getEntry();
    statusCAN_PercentBusUtilization = statusCANLayout.add("% Util.", 0).withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 0).getEntry();
    statusCAN_ReceiveErrorCount = statusCANLayout.add("RX Error Cnt", 0).withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 0).getEntry();
    statusCAN_TransmitErrorCount = statusCANLayout.add("TX Error Cnt", 0).withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 0).getEntry();
    statusCAN_TXFullCount = statusCANLayout.add("TX Full Cnt", 0).withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 0).getEntry();
  }

  private void updateXboxControllerHMI() {
    jstick1_xaxis.setDouble(m_controller.getLeftX());
    jstick1_yaxis.setDouble(m_controller.getLeftY());
    jstick2_xaxis.setDouble(m_controller.getRightX());
    jstick2_yaxis.setDouble(m_controller.getRightY());
    jstick1_click.setBoolean(m_controller.getLeftStickButton());
    jstick2_click.setBoolean(m_controller.getRightStickButton());
    leftTrigger.setDouble(m_controller.getLeftTriggerAxis());
    rightTrigger.setDouble(m_controller.getRightTriggerAxis());
    leftBumper.setBoolean(m_controller.getLeftBumper());
    rightBumper.setBoolean(m_controller.getRightBumper());
    xButton.setBoolean(m_controller.getXButton());
    yButton.setBoolean(m_controller.getYButton());
    aButton.setBoolean(m_controller.getAButton());
    bButton.setBoolean(m_controller.getBButton());
    startButton.setBoolean(m_controller.getStartButton());
    backButton.setBoolean(m_controller.getBackButton());
    String strPOV = Integer.toString(m_controller.getPOV());
    dpadPOV.setString(strPOV + " degrees");

    if(leftRumbleToggle.getBoolean(false) == true){
      m_controller.setRumble(RumbleType.kLeftRumble, leftRumbleValue.getDouble(0));
    }
    else{
      m_controller.setRumble(RumbleType.kLeftRumble,0);
    }

    if(rightRumbleToggle.getBoolean(false) == true){
      m_controller.setRumble(RumbleType.kRightRumble, rightRumbleValue.getDouble(0));
    }
    else{
      m_controller.setRumble(RumbleType.kRightRumble,0);
    }
  }
 
  private void updateNavigationHMI() {
    ahrsConnected.setBoolean(ahrs.isConnected());
    ahrsCalibrating.setBoolean(ahrs.isCalibrating());

    accelermoterXAxis.setDouble(ahrs.getRawAccelX());
    accelermoterYAxis.setDouble(ahrs.getRawAccelY());
    accelermoterZAxis.setDouble(ahrs.getRawAccelZ());

    gyroYaw.setString(ahrs.getYaw() + " Degrees");
    gyroPitch.setString(ahrs.getPitch() + " Degrees");
    gyroRoll.setString(ahrs.getRoll() + " Degrees");

    ahrsIsMoving.setBoolean(ahrs.isMoving());
    ahrsIsRotating.setBoolean(ahrs.isRotating());

    ahrsAccelX.setString(ahrs.getWorldLinearAccelX() + "");
    ahrsAccelY.setString(ahrs.getWorldLinearAccelY() + "");

    velocityX.setString(ahrs.getVelocityX() + "");
    velocityY.setString(ahrs.getVelocityX() + "");

    leftEncoderPulseCount.setString((m_leftLeadDrive.getEncoder().getPosition()*-1) + "");
    leftEncoderVelocity.setString(m_leftLeadDrive.getEncoder().getVelocity() + "");
    leftCountsPerRev.setString(m_leftLeadDrive.getEncoder().getCountsPerRevolution() + "");

    rightEncoderPulseCount.setString(m_rightLeadDrive.getEncoder().getPosition() + "");
    rightEncoderVelocity.setString(m_rightLeadDrive.getEncoder().getVelocity() + "");
    rightCountsPerRev.setString(m_rightLeadDrive.getEncoder().getCountsPerRevolution() + "");      
  }

  private void updateCommsHMI() {
    CANStatus statusCAN = RobotController.getCANStatus();
    statusCAN_BusOffCount.setDouble(statusCAN.busOffCount);
    statusCAN_PercentBusUtilization.setDouble(statusCAN.percentBusUtilization);
    statusCAN_ReceiveErrorCount.setDouble(statusCAN.receiveErrorCount);
    statusCAN_TransmitErrorCount.setDouble(statusCAN.transmitErrorCount);
    statusCAN_TXFullCount.setDouble(statusCAN.txFullCount);
  }
}
