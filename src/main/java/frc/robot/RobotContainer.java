// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;

public class RobotContainer {
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();;
    private final CommandXboxController driverController = new CommandXboxController(IOConstants.DRIVER_CONTROLLER_PORT);
    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        swerveSubsystem.setDefaultCommand(
                new SwerveJoystickCommand(
                        swerveSubsystem,
                        driverController::getLeftX,
                        driverController::getLeftY,
                        driverController::getRightY,
                        () -> !driverController.getHID().getAButton()
                )
        );

        driverController
                .a()
                .onTrue(new InstantCommand(swerveSubsystem::zeroHeading, swerveSubsystem));
    }

    public Command getAutonomousCommand() {
        // 1. Create trajectory settings
        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                .setKinematics(DriveConstants.DRIVE_KINEMATICS);

        // 2. Create trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(1, 0),
                        new Translation2d(1, -1)
                ),
                new Pose2d(2, -2, Rotation2d.fromDegrees(180)),
                config
        );

        // 3. Define PID controllers for tracking trajectory
      SwerveControllerCommand command = getSwerveControllerCommand(trajectory);

      // 5. Add some init and wrap-up logic
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose()), swerveSubsystem),
                command,
                new InstantCommand(swerveSubsystem::stopModules)
        );
    }

    private SwerveControllerCommand getSwerveControllerCommand(Trajectory trajectory) {
        PIDController xController = new PIDController(AutoConstants.PX_CONTROLLER, 0, 0);
        PIDController yController = new PIDController(AutoConstants.PY_CONTROLLER, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.P_THETA_CONTROLLER, 0, 0, AutoConstants.THETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Create command that can be run in autonomous

        return new SwerveControllerCommand(
                trajectory,
                swerveSubsystem::getPose,
                DriveConstants.DRIVE_KINEMATICS,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem
        );
    }
}
