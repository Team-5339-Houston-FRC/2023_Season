// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;

public class XboxControllerSubsystem extends SubsystemBase {

        private GenericEntry jstick1_xaxis, jstick1_yaxis, jstick2_xaxis, jstick2_yaxis, leftTrigger, rightTrigger,
                        xButton, yButton, bButton, aButton, backButton, startButton, leftBumper, rightBumper,
                        jstick1_click, jstick2_click, dpadPOV, leftRumbleToggle, leftRumbleValue, rightRumbleToggle,
                        rightRumbleValue;

        private final CommandXboxController m_driverCommandXboxController = new CommandXboxController(
                        OIConstants.kDriverControllerPort);
        private final XboxController m_driverXboxController = m_driverCommandXboxController.getHID();

        /** Creates a new DriverControllerSubsystem. */
        public XboxControllerSubsystem() {
                initHMI();
        }

        @Override
        public void periodic() {
                updateHMI();
                // This method will be called once per scheduler run
        }

        public CommandXboxController getCommandXboxController() {
                return m_driverCommandXboxController;
        }

        private void initHMI() {
                ShuffleboardTab controllerDiagTab = Shuffleboard.getTab("Controller Diag");

                ShuffleboardLayout leftJoystickLayout = controllerDiagTab.getLayout("Left Joystick", "Grid Layout")
                                .withPosition(0, 0).withSize(2, 3)
                                .withProperties(Map.of("number of columns", 1, "number of rows", 3));
                jstick1_xaxis = leftJoystickLayout.add("JStick 1 - X Axis", 0).withWidget(BuiltInWidgets.kNumberBar)
                                .withPosition(0, 0).withProperties(Map.of("min", -1, "max", 1)).getEntry();
                jstick1_yaxis = leftJoystickLayout.add("JStick 1 - Y Axis", 0).withWidget(BuiltInWidgets.kNumberBar)
                                .withPosition(0, 1).withProperties(Map.of("min", -1, "max", 1)).getEntry();
                jstick1_click = leftJoystickLayout.add("Click", false).withWidget(BuiltInWidgets.kBooleanBox)
                                .withPosition(0, 2)
                                .withProperties(Map.of("color when true", "Green", "color when false", "Black"))
                                .getEntry();

                ShuffleboardLayout rightJoystickLayout = controllerDiagTab.getLayout("Right Joystick", "Grid Layout")
                                .withPosition(2, 0).withSize(2, 3)
                                .withProperties(Map.of("number of columns", 1, "number of rows", 3));
                jstick2_xaxis = rightJoystickLayout.add("JStick 2 - X Axis", 0).withWidget(BuiltInWidgets.kNumberBar)
                                .withPosition(0, 0).withProperties(Map.of("min", -1, "max", 1)).getEntry();
                jstick2_yaxis = rightJoystickLayout.add("JStick 2 - Y Axis", 0).withWidget(BuiltInWidgets.kNumberBar)
                                .withPosition(0, 1).withProperties(Map.of("min", -1, "max", 1)).getEntry();
                jstick2_click = rightJoystickLayout.add("Click", false).withWidget(BuiltInWidgets.kBooleanBox)
                                .withPosition(0, 2)
                                .withProperties(Map.of("color when true", "Green", "color when false", "Black"))
                                .getEntry();

                leftTrigger = controllerDiagTab.add("Left Trigger", 0).withWidget(BuiltInWidgets.kNumberBar)
                                .withPosition(0, 3)
                                .withSize(2, 1).withProperties(Map.of("min", 0, "max", 1)).getEntry();

                rightTrigger = controllerDiagTab.add("Right Trigger", 0).withWidget(BuiltInWidgets.kNumberBar)
                                .withPosition(2, 3)
                                .withSize(2, 1).withProperties(Map.of("min", 0, "max", 1)).getEntry();

                leftBumper = controllerDiagTab.add("Left Bumper", false).withWidget(BuiltInWidgets.kBooleanBox)
                                .withPosition(4, 3)
                                .withSize(1, 1)
                                .withProperties(Map.of("color when true", "Green", "color when false", "Black"))
                                .getEntry();

                rightBumper = controllerDiagTab.add("Right Bumper", false).withWidget(BuiltInWidgets.kBooleanBox)
                                .withPosition(5, 3)
                                .withSize(1, 1)
                                .withProperties(Map.of("color when true", "Green", "color when false", "Black"))
                                .getEntry();

                xButton = controllerDiagTab.add("X Button", false).withWidget(BuiltInWidgets.kBooleanBox)
                                .withPosition(4, 0)
                                .withSize(1, 1)
                                .withProperties(Map.of("color when true", "Green", "color when false", "Black"))
                                .getEntry();

                yButton = controllerDiagTab.add("Y Button", false).withWidget(BuiltInWidgets.kBooleanBox)
                                .withPosition(5, 0)
                                .withSize(1, 1)
                                .withProperties(Map.of("color when true", "Green", "color when false", "Black"))
                                .getEntry();

                aButton = controllerDiagTab.add("A Button", false).withWidget(BuiltInWidgets.kBooleanBox)
                                .withPosition(4, 1)
                                .withSize(1, 1)
                                .withProperties(Map.of("color when true", "Green", "color when false", "Black"))
                                .getEntry();

                bButton = controllerDiagTab.add("B Button", false).withWidget(BuiltInWidgets.kBooleanBox)
                                .withPosition(5, 1)
                                .withSize(1, 1)
                                .withProperties(Map.of("color when true", "Green", "color when false", "Black"))
                                .getEntry();

                backButton = controllerDiagTab.add("Back", false).withWidget(BuiltInWidgets.kBooleanBox)
                                .withPosition(4, 2)
                                .withSize(1, 1)
                                .withProperties(Map.of("color when true", "Green", "color when false", "Black"))
                                .getEntry();

                startButton = controllerDiagTab.add("Start", false).withWidget(BuiltInWidgets.kBooleanBox)
                                .withPosition(5, 2)
                                .withSize(1, 1)
                                .withProperties(Map.of("color when true", "Green", "color when false", "Black"))
                                .getEntry();

                dpadPOV = controllerDiagTab.add("DPAD POV", "Not Pressed").withWidget(BuiltInWidgets.kTextView)
                                .withPosition(6, 0)
                                .getEntry();

                ShuffleboardLayout leftRumbleLayout = controllerDiagTab.getLayout("Left Rumble", "Grid Layout")
                                .withPosition(6, 2)
                                .withSize(2, 2).withProperties(Map.of("number of columns", 1, "number of rows", 2));
                leftRumbleValue = leftRumbleLayout.add("Intensity", 0).withWidget(BuiltInWidgets.kNumberSlider)
                                .withPosition(0, 0)
                                .withProperties(Map.of("min", 0, "max", 1)).getEntry();
                leftRumbleToggle = leftRumbleLayout.add("Toggle", false).withWidget(BuiltInWidgets.kToggleButton)
                                .withPosition(0, 1)
                                .getEntry();

                ShuffleboardLayout rightRumbleLayout = controllerDiagTab.getLayout("Right Rumble", "Grid Layout")
                                .withPosition(8, 2)
                                .withSize(2, 2).withProperties(Map.of("number of columns", 1, "number of rows", 2));
                rightRumbleValue = rightRumbleLayout.add("Intensity", 0).withWidget(BuiltInWidgets.kVoltageView)
                                .withPosition(0, 0)
                                .withProperties(Map.of("min", 0, "max", 1)).getEntry();
                rightRumbleToggle = rightRumbleLayout.add("Toggle", false).withWidget(BuiltInWidgets.kToggleButton)
                                .withPosition(0, 1).getEntry();
        }

        private void updateHMI() {
                jstick1_xaxis.setDouble(m_driverXboxController.getLeftX());
                jstick1_yaxis.setDouble(m_driverXboxController.getLeftY());
                jstick2_xaxis.setDouble(m_driverXboxController.getRightX());
                jstick2_yaxis.setDouble(m_driverXboxController.getRightY());
                jstick1_click.setBoolean(m_driverXboxController.getLeftStickButton());
                jstick2_click.setBoolean(m_driverXboxController.getRightStickButton());
                leftTrigger.setDouble(m_driverXboxController.getLeftTriggerAxis());
                rightTrigger.setDouble(m_driverXboxController.getRightTriggerAxis());
                leftBumper.setBoolean(m_driverXboxController.getLeftBumper());
                rightBumper.setBoolean(m_driverXboxController.getRightBumper());
                xButton.setBoolean(m_driverXboxController.getXButton());
                yButton.setBoolean(m_driverXboxController.getYButton());
                aButton.setBoolean(m_driverXboxController.getAButton());
                bButton.setBoolean(m_driverXboxController.getBButton());
                startButton.setBoolean(m_driverXboxController.getStartButton());
                backButton.setBoolean(m_driverXboxController.getBackButton());
                String strPOV = Integer.toString(m_driverXboxController.getPOV());
                dpadPOV.setString(strPOV + " degrees");

                if (leftRumbleToggle.getBoolean(false) == true) {
                        m_driverXboxController.setRumble(RumbleType.kLeftRumble, leftRumbleValue.getDouble(0));
                } else {
                        m_driverXboxController.setRumble(RumbleType.kLeftRumble, 0);
                }

                if (rightRumbleToggle.getBoolean(false) == true) {
                        m_driverXboxController.setRumble(RumbleType.kRightRumble, rightRumbleValue.getDouble(0));
                } else {
                        m_driverXboxController.setRumble(RumbleType.kRightRumble, 0);
                }
        }
}
