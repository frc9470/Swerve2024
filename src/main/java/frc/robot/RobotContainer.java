// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.IOConstants;

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
    return Commands.print("No autonomous command configured");
  }
}
