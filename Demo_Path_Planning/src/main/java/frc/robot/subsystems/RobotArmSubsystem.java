// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class RobotArmSubsystem extends SubsystemBase {

  PneumaticHub pneumaticHub = new PneumaticHub(DriveConstants.kPneumaticHubCANID);
  Solenoid clawSolenoid = pneumaticHub.makeSolenoid(1);
  Compressor compressor = pneumaticHub.makeCompressor();
  CANSparkMax m_telescopeMotor = new CANSparkMax(DriveConstants.kRetractMotorCANID, MotorType.kBrushless);
  CANSparkMax m_pivotMotor = new CANSparkMax(DriveConstants.kPivotMotorCANID, MotorType.kBrushless);
  RelativeEncoder m_pivotEncoder = m_pivotMotor.getEncoder();

  private final RelativeEncoder m_telescopeEncoder = m_telescopeMotor.getEncoder();

  private GenericEntry telescopeEncoderHMI, pivotEncoderHMI;

  /** Creates a new RobotArmSubsystem. */
  public RobotArmSubsystem() {
    compressor.enableAnalog(100, 120);

    m_telescopeMotor.restoreFactoryDefaults();
    m_telescopeMotor.setIdleMode(IdleMode.kBrake);
    m_telescopeEncoder.setPositionConversionFactor(1);
    m_telescopeEncoder.setVelocityConversionFactor(1);
    m_telescopeEncoder.setPosition(0);

    m_pivotMotor.restoreFactoryDefaults();
    m_pivotMotor.setIdleMode(IdleMode.kBrake);
    m_pivotEncoder.setPositionConversionFactor(1);
    m_pivotEncoder.setVelocityConversionFactor(1);
    m_pivotEncoder.setPosition(0);

    initHMI();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateHMI();
  }

  public void setSolenoid(boolean state) {
    clawSolenoid.set(state);
  }



  public void controlArm(double telescopeRetract, double telescopeExtend, double pivot) {
    double telescope = .5 * (telescopeRetract + (telescopeExtend * -1));

    if(telescope >= .1 || telescope <= -.1)
    {
      if(this.m_telescopeEncoder.getPosition() <= 3 && telescope > 0){
        m_telescopeMotor.set(telescope);
      }
      else if(this.m_telescopeEncoder.getPosition() >= -62 && telescope < 0)
      {
        m_telescopeMotor.set(telescope);
      }
      else {
        m_telescopeMotor.set(0);
      }
    }
    else {
      m_telescopeMotor.set(0);
    }

    if(pivot >= .1 || pivot <= -.1)
    {
      m_pivotMotor.set(pivot);
    }
    else {
      m_pivotMotor.set(0);
    }
  }

  private void initHMI(){
    ShuffleboardTab armDiagTab = Shuffleboard.getTab("Claw Diag");

    ShuffleboardLayout encoderLayout = armDiagTab.getLayout("Encoders", "Grid Layout").withPosition(0, 0).withSize(3, 2)
        .withProperties(Map.of("number of columns", 2, "number of rows", 3));
        telescopeEncoderHMI = encoderLayout.add("Telescope Encoder", "").withWidget(BuiltInWidgets.kTextView).withPosition(0, 0).getEntry();
        pivotEncoderHMI = encoderLayout.add("Pivot Encoder", "").withWidget(BuiltInWidgets.kTextView).withPosition(1, 0).getEntry();



  }
  private void updateHMI(){
    telescopeEncoderHMI.setString(String.format("%.3f", this.m_telescopeEncoder.getPosition()) + " p");
    pivotEncoderHMI.setString(String.format("%.3f", this.m_pivotEncoder.getPosition()) + " p");
  }
}
