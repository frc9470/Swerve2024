package frc.robot.commands;


import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

public class SwerveJoystickCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpeedSupplier, ySpeedSupplier, rotSpeedSupplier;
    private final Supplier<Boolean> fieldRelativeSupplier;
    private final SlewRateLimiter xLimiter, yLimiter, rotLimiter;

    public SwerveJoystickCommand(SwerveSubsystem swerveSubsystem,
                                 Supplier<Double> xSpeedSupplier,
                                 Supplier<Double> ySpeedSupplier,
                                 Supplier<Double> rotSpeedSupplier,
                                 Supplier<Boolean> fieldRelativeSupplier) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.rotSpeedSupplier = rotSpeedSupplier;
        this.fieldRelativeSupplier = fieldRelativeSupplier;
        this.xLimiter = new SlewRateLimiter(DriveConstants.TELE_DRIVE_MAX_ACCELERATION);
        this.yLimiter = new SlewRateLimiter(DriveConstants.TELE_DRIVE_MAX_ACCELERATION);
        this.rotLimiter = new SlewRateLimiter(DriveConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xSpeed = xSpeedSupplier.get();
        double ySpeed = ySpeedSupplier.get();
        double rotSpeed = rotSpeedSupplier.get();

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > IOConstants.DEADBAND ? xSpeed : 0;
        ySpeed = Math.abs(ySpeed) > IOConstants.DEADBAND ? ySpeed : 0;
        rotSpeed = Math.abs(rotSpeed) > IOConstants.DEADBAND ? rotSpeed : 0;

        // 3. Apply acceleration limiters and make driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.TELE_DRIVE_MAX_SPEED_METERS_PER_SECOND;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.TELE_DRIVE_MAX_SPEED_METERS_PER_SECOND;
        rotSpeed = rotLimiter.calculate(rotSpeed) * DriveConstants.TELE_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldRelativeSupplier.get()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, swerveSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
        }

        // 5. Convert chassis speeds to module states
        SwerveModuleState[] moduleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        // 6. Send module states to modules
        swerveSubsystem.setModuleStates(moduleStates);

    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

}
