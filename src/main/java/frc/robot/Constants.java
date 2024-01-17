package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class ModuleConstants{
        // TODO: Adjust values
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
        public static final double DRIVE_MOTOR_GEAR_RATIO = 1 / 6.75;
        public static final double TURNING_MOTOR_GEAR_RATIO = 1 / (150 / 7.0);
        public static final double DRIVE_ENCODER_ROT_TO_METER = DRIVE_MOTOR_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS;
        public static final double TURNING_ENCODER_ROT_TO_RAD = TURNING_MOTOR_GEAR_RATIO * 2 * Math.PI;
        public static final double DRIVE_ENCODER_RPM_TO_MPS = DRIVE_ENCODER_ROT_TO_METER / 60;
        public static final double TURNING_ENCODER_RPM_TO_RPS = TURNING_ENCODER_ROT_TO_RAD / 60;
        public static final double P_TURNING = 0.5;
    }

    public static final class DriveConstants {

        public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(21);
        public static final double WHEEL_BASE_METERS = Units.inchesToMeters(25.5);
        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2),
                new Translation2d(WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
                new Translation2d(-WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2),
                new Translation2d(-WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2));

        public static final int FRONT_LEFT_DRIVE_MOTOR_PORT = 8;
        public static final int BACK_LEFT_DRIVE_MOTOR_PORT = 2;
        public static final int FRONT_RIGHT_DRIVE_MOTOR_PORT = 6;
        public static final int BACK_RIGHT_DRIVE_MOTOR_PORT = 4;

        public static final int FRONT_LEFT_TURNING_MOTOR_PORT = 7;
        public static final int BACK_LEFT_TURNING_MOTOR_PORT = 1;
        public static final int FRONT_RIGHT_TURNING_MOTOR_PORT = 5;
        public static final int BACK_RIGHT_TURNING_MOTOR_PORT = 3;

        public static final boolean FRONT_LEFT_TURNING_ENCODER_REVERSED = true;
        public static final boolean BACK_LEFT_TURNING_ENCODER_REVERSED = true;
        public static final boolean FRONT_RIGHT_TURNING_ENCODER_REVERSED = true;
        public static final boolean BACK_RIGHT_TURNING_ENCODER_REVERSED = true;

        public static final boolean FRONT_LEFT_DRIVE_ENCODER_REVERSED = true;
        public static final boolean BACK_LEFT_DRIVE_ENCODER_REVERSED = true;
        public static final boolean FRONT_RIGHT_DRIVE_ENCODER_REVERSED = false;
        public static final boolean BACK_RIGHT_DRIVE_ENCODER_REVERSED = false;

        public static final int FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_PORT = 0;
        public static final int BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_PORT = 2;
        public static final int FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_PORT = 1;
        public static final int BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_PORT = 3;

        public static final boolean FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;
        public static final boolean BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;
        public static final boolean FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;
        public static final boolean BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;

        public static final double FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD = -0.254;
        public static final double BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD = -1.252;
        public static final double FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD = -1.816;
        public static final double BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD = -4.811;

        public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 5;
        public static final double PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * 2 * Math.PI;

        public static final double TELE_DRIVE_MAX_SPEED_METERS_PER_SECOND = PHYSICAL_MAX_SPEED_METERS_PER_SECOND / 4;
        public static final double TELE_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND / 4;
        public static final double TELE_DRIVE_MAX_ACCELERATION = 3;
        public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION= 3;
    }

    public static final class IOConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int DRIVER_Y_AXIS = 1;
        public static final int DRIVER_X_AXIS = 0;
        public static final int DRIVER_ROTATE_AXIS = 4;
        public static final int DRIVER_FIELD_CENTRIC_BUTTON = 1;
        public static final double DEADBAND = 0.05;
    }

    public static final class AutoConstants {
        public static final double MAX_SPEED_METERS_PER_SECOND = DriveConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND / 4;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND =
                DriveConstants.PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND / 10;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = Math.PI / 4;
        public static final double PX_CONTROLLER = 1.5;
        public static final double PY_CONTROLLER = 1.5;
        public static final double P_THETA_CONTROLLER = 3;

        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = //
                new TrapezoidProfile.Constraints(
                        MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
                        MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
    }

}
