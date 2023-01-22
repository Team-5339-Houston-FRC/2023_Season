// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;


public class AHRSSubsystem extends SubsystemBase {

  // Maps for Navigation Diag Screen
  private GenericEntry ahrsConnected, ahrsCalibrating, accelermoterXAxis, accelermoterYAxis, accelermoterZAxis,
      ahrsIsMoving, ahrsIsRotating, ahrsAccelX, ahrsAccelY, gyroYaw, gyroPitch, gyroRoll, velocityX, velocityY;

  private final AHRS m_ahrs = new AHRS(SPI.Port.kMXP);

  /** Creates a new AHRSSubsystem. */
  public AHRSSubsystem() {
    initHMI();
  }

  @Override
  public void periodic() {
    updateHMI();
    // This method will be called once per scheduler run
  }

  public AHRS getAHRS(){
    return m_ahrs;
  }

  private void initHMI() {
    ShuffleboardTab driveDiagTab = Shuffleboard.getTab("AHRS Diag");

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
    
  }

  private void updateHMI() {
    ahrsConnected.setBoolean(m_ahrs.isConnected());
    ahrsCalibrating.setBoolean(m_ahrs.isCalibrating());

    accelermoterXAxis.setDouble(m_ahrs.getRawAccelX());
    accelermoterYAxis.setDouble(m_ahrs.getRawAccelY());
    accelermoterZAxis.setDouble(m_ahrs.getRawAccelZ());

    gyroYaw.setString(m_ahrs.getYaw() + " Degrees");
    gyroPitch.setString(m_ahrs.getPitch() + " Degrees");
    gyroRoll.setString(m_ahrs.getRoll() + " Degrees");

    ahrsIsMoving.setBoolean(m_ahrs.isMoving());
    ahrsIsRotating.setBoolean(m_ahrs.isRotating());

    ahrsAccelX.setString(m_ahrs.getWorldLinearAccelX() + "");
    ahrsAccelY.setString(m_ahrs.getWorldLinearAccelY() + "");

    velocityX.setString(m_ahrs.getVelocityX() + "");
    velocityY.setString(m_ahrs.getVelocityX() + "");
   
  }
}
