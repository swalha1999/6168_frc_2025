package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 8;

        public static final COTSTalonFXSwerveConstants chosenModule = COTSTalonFXSwerveConstants.SDS.MK4i
                .KrakenX60DriveFalcon500Angle(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = 0.56;
        public static final double wheelBase = 0.69;
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32;
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.2672;
        /** Radians per Second */
        public static final double maxAngularVelocity = 5.0;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 32;
            public static final int angleMotorID = 30;
            public static final int canCoderID = 31;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(160.048 + 180);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 42;
            public static final int angleMotorID = 40;
            public static final int canCoderID = 41;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(14.414 + 180);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 22;
            public static final int angleMotorID = 20;
            public static final int canCoderID = 21;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-117.509 + 180);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 12;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-74.091 + 180);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class TelescopeConstants {
        public static final double max_extension_ticks = 80.0;
        public static final double min_extension_ticks = 0.0;
    }

    public static final class PivotConstants {
        public static final double max_pivot_ticks = 14.0;
        public static final double min_pivot_ticks = 0.0;
    }

    public static final class EndEffectorConstants {
        public static final int ANGLE_MOTOR_ID = 53; // Change ID as needed
        public static final int INTAKE_MOTOR_ID = 61; // Change ID as needed

        // Angle Motor Constants
        public static final double ANGLE_kP = 0.1;
        public static final double ANGLE_kI = 0.0;
        public static final double ANGLE_kD = 0.0;
        public static final double ANGLE_kFF = 0.0;

        // Angle Constraints
        public static final double MAX_ANGLE_POSITION = 90.0; // degrees
        public static final double MIN_ANGLE_POSITION = 0.0; // degrees

        // Intake Motor Constants
        public static final double INTAKE_SPEED = 0.7; // 70% speed for intake
        public static final double EJECT_SPEED = -0.7; // 70% speed for ejecting

        // Current Limits
        public static final int ANGLE_CURRENT_LIMIT = 30; // amps
        public static final int INTAKE_CURRENT_LIMIT = 40; // amps

        public static final double ANGLE_OUTPUT_MIN = -1.0;
        public static final double ANGLE_OUTPUT_MAX = 1.0;

        public static final double ANGLE_TOLERANCE = 1.0;
    }

    public static final class LimelightConstants {
        // public static final String[] LIMELIGHT_NAMES = {};
        public static final String[] LIMELIGHT_NAMES = { "limelight", "limelight-orsan" };

        public static final double MAX_ANGULAR_VELOCITY_REJECT = 720.0; // degrees per second
    }

    public static final class TargetPostioons {
        public static final class Red {

            // Left goals
            public static final Pose2d[] CORAL_GOALS_LEFT = {
                    new Pose2d(14.36, 3.81, Rotation2d.fromDegrees(180)), // 7L
                    new Pose2d(13.47, 2.75, Rotation2d.fromDegrees(120)), // 6L
                    new Pose2d(14.01, 4.95, Rotation2d.fromDegrees(240)), // 8L
                    new Pose2d(12.69, 5.27, Rotation2d.fromDegrees(300)), // 9L
                    new Pose2d(11.81, 4.01, Rotation2d.fromDegrees(0)), // 10L
                    new Pose2d(12.41, 2.92, Rotation2d.fromDegrees(60)) // 11L
            };

            // Right goals
            public static final Pose2d[] CORAL_GOALS_RIGHT = {
                    new Pose2d(14.37, 4.16, Rotation2d.fromDegrees(180)), // 7R
                    new Pose2d(13.74, 2.90, Rotation2d.fromDegrees(120)), // 6R
                    new Pose2d(13.68, 5.16, Rotation2d.fromDegrees(240)), // 8R
                    new Pose2d(12.44, 5.13, Rotation2d.fromDegrees(300)), // 9R
                    new Pose2d(11.81, 3.73, Rotation2d.fromDegrees(0)), // 10R
                    new Pose2d(12.71, 2.75, Rotation2d.fromDegrees(60)) // 11R
            };

            public static final Pose2d[] FEEDER_GOALS = {
                    new Pose2d(16.475, 0.931, Rotation2d.fromDegrees(-55)), 
                    new Pose2d(16.366, 6.960, Rotation2d.fromDegrees(55)), // 7R
                    
            };

            public static final Pose2d[] ALRGE_PROCCESOR_GOALS = {
                    new Pose2d(14.36, 3.73, Rotation2d.fromDegrees(180)), // 7L
            };

        };

        public static final class Blue {

            // Left goals
            public static final Pose2d[] CORAL_GOALS_LEFT = {
                new Pose2d(3.19, 4.21, Rotation2d.fromDegrees(0)), // 18L
                    
            };

            // Right goals
            public static final Pose2d[] CORAL_GOALS_RIGHT = {
                new Pose2d(3.19, 3.89, Rotation2d.fromDegrees(0)), // 18R
    
            };

            public static final Pose2d[] FEEDER_GOALS = {
                    new Pose2d(1.155, 1.054, Rotation2d.fromDegrees(-125)), 
                    new Pose2d(16.366, 6.945, Rotation2d.fromDegrees(125)), // 7R
                    
            };

            public static final Pose2d[] ALRGE_PROCCESOR_GOALS = {
                    new Pose2d(1.133, 3.73, Rotation2d.fromDegrees(180)), // 7L
            };

        };
    }
}
