package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;


public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            FRONT_LEFT_DRIVE_MOTOR_PORT,
            FRONT_LEFT_TURNING_MOTOR_PORT,
            FRONT_LEFT_DRIVE_ENCODER_REVERSED,
            FRONT_LEFT_TURNING_ENCODER_REVERSED,
            FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_PORT,
            FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_REVERSED,
            FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD
    );

    private final SwerveModule frontRight = new SwerveModule(
            FRONT_RIGHT_DRIVE_MOTOR_PORT,
            FRONT_RIGHT_TURNING_MOTOR_PORT,
            FRONT_RIGHT_DRIVE_ENCODER_REVERSED,
            FRONT_RIGHT_TURNING_ENCODER_REVERSED,
            FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_PORT,
            FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_REVERSED,
            FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD
    );

    private final SwerveModule backLeft = new SwerveModule(
            BACK_LEFT_DRIVE_MOTOR_PORT,
            BACK_LEFT_TURNING_MOTOR_PORT,
            BACK_LEFT_DRIVE_ENCODER_REVERSED,
            BACK_LEFT_TURNING_ENCODER_REVERSED,
            BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_PORT,
            BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_REVERSED,
            BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD
    );

    private final SwerveModule backRight = new SwerveModule(
            BACK_RIGHT_DRIVE_MOTOR_PORT,
            BACK_RIGHT_TURNING_MOTOR_PORT,
            BACK_RIGHT_DRIVE_ENCODER_REVERSED,
            BACK_RIGHT_TURNING_ENCODER_REVERSED,
            BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_PORT,
            BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_REVERSED,
            BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD
    );

    private AHRS gyro = new AHRS(SPI.Port.kMXP);

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception ignored){

            }
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading(){
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Heading", getHeading());
    }

    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, TELE_DRIVE_MAX_SPEED_METERS_PER_SECOND);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}
