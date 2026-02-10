package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.BisonLib.BaseProject.Swerve.SwerveConfig;

public class Constants {
    public static final double g = 9.81;
    public static final double BUMP_THRESHOLD = 1.0;
    public static final class Swerve {
        
    public static final SwerveConfig prod_2026_Config =
            new SwerveConfig(
            // module offsets
                0.268798828125 + 0.5, // front right
                -0.572266, // front left
                0.0388, // back left
                -0.137939 + 0.5, // back right
            // drive gear ratio
            6.026785714285714,
            // max speed ft/sec
            Units.metersToFeet(4.9),
            // wheel diameter
            4 * Math.PI, 
            // turn wheel kp
            70, 
            // wheelbase and track width
            21.5, 21.5,
            // turn gear ratio
            26.0, 
            // inverted drive motor
            false, 
            0.01, 
            0,
            // tune stator limit; supply limit doesn't get applied
            120, 50, 
            // tune velocity pid and ff
            0, 0, 0, 
            0.4001,0.115,
            0.0,//0.038,
            0.14, 1/(30*360/(10435.649414+359.033203)));
        public static final SwerveConfig QC_Config =
            new SwerveConfig(
                //must find all offsets
            -0.206298828125, // front right
            0.182373046875 + 0.5, // front left
            -0.04443359375 + 0.5, // back left
            0.076904296875, // back right
            // ask about this
            6.12,
            // has to be tuned
            16.5,
            // has to be tuned
            4 * Math.PI, 
            // must find this kp
            70, 
            // must find wheelbase and track width
            23.75, 23.75, 
            150.0/7, 
            true, 
            // must tune this
            0.01, 
            // find this with field
            0, 
            // tune stator limit; supply limit doesn't get applied
            90, 40, 
            // tune velocity pid and ff
            //0.145 ks
            0, 0.145, 0, 
            0.38, 0.13,
            0, //0.003216875
            -0.15032, 1/(((360*30)-180)/11468.4));
            //kp, kv, ka, kS
            // kp=0.3, kv=0.13 is very good combo
        public static final SwerveConfig QBConfig = 
                new SwerveConfig(-0.4625, 
                -0.1408, 
                 0.018799, 
                -0.068115, 6.12, 16.5,
                4 * Math.PI, 45, 23.75, 
                23.75, 150.0/7, true, 
                0.006, 
                0, 90, 
                40, 0, 0, 
                0, 0.25, 
                0.11, 0, 0.2, 14040/14053.447266);
        public static final SwerveConfig production2025Config =
                new SwerveConfig(
                    //must find all offsets
                0.0457 + 0.5, // front right
                -0.064209, // front left
                0.061668, // back left
                -0.14446, // back right
                 8.14, Units.metersToFeet(3.6), 4 * Math.PI, 
                 // must find this kp
                 70, 
                 // must find wheelbase and track width
                 23.75, 23.75, 150.0/7, true, 
                 // must tune this
                 0.01, 
                 // find this with field
                 0, 
                 // tune stator limit; supply limit doesn't get applied
                 90, 40, 
                 // tune velocity pid and ff
                 0, 0.145, 0, 
                 0.05, 0.12,0,  0.2, 1.0/1.003344);
                 //0.99622314806

        public static final Map<String, SwerveConfig> ROBOT_MAP = new HashMap<String, SwerveConfig>() {
            {
                put("QB", QBConfig);
                put("Production_2025", production2025Config);
                put("QC", QC_Config);
                put("Prod_2026", prod_2026_Config);
            }
        };
        

        // CHOOSE WHICH ROBOT YOU'RE USING
        public static final SwerveConfig CHOSEN_CONSTANTS = ROBOT_MAP.get("Prod_2026");

        // miscellaneous constants
        public static final double MAX_SPEED_METERS_PER_SECONDS_TELEOP = CHOSEN_CONSTANTS.maxSpeedMetersPerSec;
        public static final double MAX_TRACKABLE_SPEED_METERS_PER_SECOND = 4;
        public static final double MAX_ANGULAR_SPEED_RAD_PER_SECOND = CHOSEN_CONSTANTS.maxAngularSpeedRadPerSec;
        public static final double TURNING_GEAR_RATIO = CHOSEN_CONSTANTS.turningGearRatio;
        public static final double DRIVING_GEAR_RATIO = CHOSEN_CONSTANTS.drivingGearRatio;
        public static final double WHEEL_CIRCUMFERENCE_METERS = CHOSEN_CONSTANTS.wheelCircumferenceMeters;
        public static final double TURN_WHEEL_KP = CHOSEN_CONSTANTS.turnWheelKP;
        public static final double TURN_WHEEL_KS = CHOSEN_CONSTANTS.turnWheelKS;
        public static final double TURN_WHEEL_KD = CHOSEN_CONSTANTS.turnWheelKD;
        public static final double ROBOT_ROTATION_KP = 0.008;
        public static final double MAX_WHEEL_ROTATIONAL_SPEED = CHOSEN_CONSTANTS.maxWheelRotationalSpeed;
        public static final double GYRO_DRIFT_COMPENSATION = CHOSEN_CONSTANTS.gyroDriftCompensation;
        public static final double SKEW_COMPENSATION_RATE = -0.07;

        public static final double MAX_SKID_ACCEL = 100;
        public static final double MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED = CHOSEN_CONSTANTS.maxAngularAccelerationRadPerSecondSquared;
        public static final double DISCRETIZE_TIMESTAMP = 0.02;
        public static final int ODOMETRY_UPDATE_RATE_HZ_INTEGER = 200;
        public static final boolean MODULE_IS_INVERTED = CHOSEN_CONSTANTS.driveMotorInverted;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQ = 33.5;
        public static final double MAX_WHEEL_TRACTION_METERS_PER_SECOND_SQ = 15;
        public static final double MAX_ACCEL_METERS_PER_SECOND_SQ_AUTOALIGN = 10;
        public static final double SUPPLY_CURRENT_LIMIT = CHOSEN_CONSTANTS.supplyCurrentLimit;
        public static final double STATOR_CURRENT_LIMIT = CHOSEN_CONSTANTS.statorCurrentLimit;

        // configs for drive wheel (closed-loop velocity control)
        public static final double DRIVE_WHEEL_KP = CHOSEN_CONSTANTS.driveWheelKP;
        public static final double DRIVE_WHEEL_KV = CHOSEN_CONSTANTS.driveWheelKV;
        public static final double DRIVE_WHEEL_KS = CHOSEN_CONSTANTS.driveWheelKS;
        public static final double DRIVE_WHEEL_KA = CHOSEN_CONSTANTS.driveWheelKA;


        public static final int GYRO_ID = 8;
        // front right wheel
        public static final int FRONT_RIGHT_DRIVE_ID = 13;
        public static final int FRONT_RIGHT_TURN_ID = 12;
        public static final int FRONT_RIGHT_CANCODER_ID = 11;
        public static final double FRONT_RIGHT_ABS_ENCODER_OFFSET_ROTATIONS = CHOSEN_CONSTANTS.frontRightOffset;

        // front left wheel
        public static final int FRONT_LEFT_DRIVE_ID = 23;
        public static final int FRONT_LEFT_TURN_ID = 22;
        public static final int FRONT_LEFT_CANCODER_ID = 21;
        public static final double FRONT_LEFT_ABS_ENCODER_OFFSET_ROTATIONS = CHOSEN_CONSTANTS.frontLeftOffset;

        // back left wheel
        public static final int BACK_LEFT_DRIVE_ID = 33;
        public static final int BACK_LEFT_TURN_ID = 32;
        public static final int BACK_LEFT_CANCODER_ID = 31;
        public static final double BACK_LEFT_ABS_ENCODER_OFFSET_ROTATIONS = CHOSEN_CONSTANTS.backLeftOffset;

        // back right wheel
        public static final int BACK_RIGHT_DRIVE_ID = 43;
        public static final int BACK_RIGHT_TURN_ID = 42;
        public static final int BACK_RIGHT_CANCODER_ID = 41;
        public static final double BACK_RIGHT_ABS_ENCODER_OFFSET_ROTATIONS = CHOSEN_CONSTANTS.backRightOffset;

        public static final TrapezoidProfile.Constraints TRAPEZOID_THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED_RAD_PER_SECOND, MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED);

        public static final double TRACK_WIDTH_METERS = CHOSEN_CONSTANTS.trackWidthMeters;
        public static final double WHEEL_BASE_METERS = CHOSEN_CONSTANTS.wheelBaseMeters;

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                CHOSEN_CONSTANTS.frontRightTranslation, // Front right wheel
                CHOSEN_CONSTANTS.frontLeftTranslation, // Front left wheel
                CHOSEN_CONSTANTS.backLeftTranslation, // Back left wheel
                CHOSEN_CONSTANTS.backRightTranslation); // Back right wheel

        public static final Translation2d FRONT_LEFT_TRANSLATION = CHOSEN_CONSTANTS.frontLeftTranslation;
        public static final Translation2d FRONT_RIGHT_TRANSLATION = CHOSEN_CONSTANTS.frontRightTranslation;
        public static final Translation2d BACK_LEFT_TRANSLATION = CHOSEN_CONSTANTS.backLeftTranslation;
        public static final Translation2d BACK_RIGHT_TRANSLATION = CHOSEN_CONSTANTS.backRightTranslation;
    }
}