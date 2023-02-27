// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.AHRSSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RobotArmSubsystem;
import frc.robot.subsystems.XboxControllerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.io.IOException;
import java.nio.file.Path;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems
        private final XboxControllerSubsystem m_driverControllerSubsystem = new XboxControllerSubsystem();
        private final AHRSSubsystem m_ahrsSubsystem = new AHRSSubsystem();
        private final DriveSubsystem m_robotDriveSubSystem = new DriveSubsystem(m_ahrsSubsystem);
        private final RobotArmSubsystem m_robotArmSubsystem = new RobotArmSubsystem();
        // private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem();

        private final CommandXboxController m_driverCommandController = m_driverControllerSubsystem
                        .getCommandXboxController();

        String trajectoryJSON = "paths/output/test.path.1.wpilib.json";
        Trajectory trajectory = new Trajectory();

        ;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                try {
                        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
                        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
                        Path trajectoryPath1 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
                        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath1);
                } catch (IOException ex) {
                        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
                }
                // Configure the button bindings
                configureButtonBindings();
                // Set the max output percent for the motors
                m_robotDriveSubSystem.setMaxOutput(0.3);

                // Configure default commands
                // Set the default drive command to single-stick arcade drive
                m_robotDriveSubSystem.setDefaultCommand(
                                // A single-stick arcade command, controlled by the left stick.
                                new RunCommand(
                                                () -> m_robotDriveSubSystem.arcadeDrive(
                                                                m_driverCommandController.getLeftY() * -1,
                                                                m_driverCommandController.getLeftX() * -1),
                                                m_robotDriveSubSystem)
                                                .alongWith(new RunCommand(() -> m_robotArmSubsystem
                                                                .controlArm(m_driverCommandController.getLeftTriggerAxis(),
                                                                m_driverCommandController.getRightTriggerAxis(),
                                                                                m_driverCommandController.getRightX()),
                                                                m_robotArmSubsystem)));
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
         * subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
         * passing it to a
         * {@link JoystickButton}.
         */
        private void configureButtonBindings() {
                // Drive at half speed when the right bumper is held
                m_driverCommandController.a()
                                .onTrue(new InstantCommand(() -> m_robotDriveSubSystem.setMaxOutput(0.5)))
                                .onFalse(new InstantCommand(() -> m_robotDriveSubSystem.setMaxOutput(0.3)));

                // Auto-Balance when the left bumper is held
                m_driverCommandController.y()
                                .whileTrue(new RunCommand(
                                                () -> m_robotDriveSubSystem.autoBalance(),
                                                m_robotDriveSubSystem));

                m_driverCommandController.x()
                                .onTrue(new InstantCommand(
                                                () -> m_robotDriveSubSystem
                                                                .resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))));
                m_driverCommandController.rightBumper()
                                .onTrue(new RunCommand(
                                                () -> m_robotArmSubsystem.setSolenoid(false),
                                                m_robotArmSubsystem));
                m_driverCommandController.leftBumper()
                                .onTrue(new RunCommand(
                                                () -> m_robotArmSubsystem.setSolenoid(true),
                                                m_robotArmSubsystem));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // // Create a voltage constraint to ensure we don't accelerate too fast
                // var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                // new SimpleMotorFeedforward(
                // DriveConstants.ksVolts,
                // DriveConstants.kvVoltSecondsPerMeter,
                // DriveConstants.kaVoltSecondsSquaredPerMeter),
                // DriveConstants.kDriveKinematics,
                // 6);

                // // Create config for trajectory
                // TrajectoryConfig config = new TrajectoryConfig(
                // AutoConstants.kMaxSpeedMetersPerSecond,
                // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // // Add kinematics to ensure max speed is actually obeyed
                // .setKinematics(DriveConstants.kDriveKinematics)
                // // Apply the voltage constraint
                // .addConstraint(autoVoltageConstraint);

                // /*
                // * // An example trajectory to follow. All units in meters.
                // * Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // * // Start at the origin facing the +X direction
                // * new Pose2d(0, 0, new Rotation2d(0)),
                // * // Pass through these two interior waypoints, making an 's' curve path
                // * List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // * // End 3 meters straight ahead of where we started, facing forward
                // * new Pose2d(3, 0, new Rotation2d(0)),
                // * // Pass config
                // * config);
                // */

                // RamseteCommand ramseteCommand = new RamseteCommand(
                // trajectory,
                // m_robotDriveSubSystem::getPose,
                // new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                // new SimpleMotorFeedforward(
                // DriveConstants.ksVolts,
                // DriveConstants.kvVoltSecondsPerMeter,
                // DriveConstants.kaVoltSecondsSquaredPerMeter),
                // DriveConstants.kDriveKinematics,
                // m_robotDriveSubSystem::getWheelSpeeds,
                // new PIDController(DriveConstants.kPDriveVel, 0, 0),
                // new PIDController(DriveConstants.kPDriveVel, 0, 0),
                // // RamseteCommand passes volts to the callback
                // m_robotDriveSubSystem::tankDriveVolts,
                // m_robotDriveSubSystem);
                // RamseteCommand ramseteCommand1 = new RamseteCommand(
                // trajectory,
                // m_robotDriveSubSystem::getPose,
                // new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                // new SimpleMotorFeedforward(
                // DriveConstants.ksVolts,
                // DriveConstants.kvVoltSecondsPerMeter,
                // DriveConstants.kaVoltSecondsSquaredPerMeter),
                // DriveConstants.kDriveKinematics,
                // m_robotDriveSubSystem::getWheelSpeeds,
                // new PIDController(DriveConstants.kPDriveVel, 0, 0),
                // new PIDController(DriveConstants.kPDriveVel, 0, 0),
                // // RamseteCommand passes volts to the callback
                // m_robotDriveSubSystem::tankDriveVolts,
                // m_robotDriveSubSystem);

                // // Reset odometry to the starting pose of the trajectory.
                // m_robotDriveSubSystem.resetOdometry(trajectory.getInitialPose());

                // // Run path following command, then stop at the end.
                // return ramseteCommand.andThen(ramseteCommand1)
                // .andThen(() -> m_robotDriveSubSystem.tankDriveVolts(0, 0));
                return null;
        }
}
