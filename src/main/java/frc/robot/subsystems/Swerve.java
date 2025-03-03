package frc.robot.subsystems;

import java.util.Optional;

// WPILib Imports
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

// CTRE Imports
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

// PathPlanner Imports


// Local Imports
import frc.lib.util.LimelightHelpers;
import frc.robot.SwerveModule;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
    public SwerveDrivePoseEstimator poseEstimator;
    public SwerveDrivePoseEstimator autoPoseEstimator;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    RobotConfig config;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroYaw(),
                getModulePositions(), new Pose2d());
        
        autoPoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroYaw(),
                getModulePositions(), new Pose2d());

        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 0.99));
        autoPoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 0.99));
        
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

         AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(   10, 0, 0.0), // Translation PID constants
                    new PIDConstants(1, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getHeading())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativChassisSpeeds) {
		SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics
				.toSwerveModuleStates(robotRelativChassisSpeeds);
		setModuleStates(swerveModuleStates);
	}

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
		return Constants.Swerve.swerveKinematics.toChassisSpeeds(
				mSwerveMods[0].getState(),
				mSwerveMods[1].getState(),
				mSwerveMods[2].getState(),
				mSwerveMods[3].getState());
	}

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Pose2d getAutoPose() {
        return autoPoseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
        System.out.println("Pose Reset to " + pose);
    }

    public void setAutoPose(Pose2d pose) {
        autoPoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
        System.out.println("Auto Pose Reset to " + pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public Rotation2d getBlueHeading() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                return getHeading().rotateBy(Rotation2d.fromDegrees(180));
            }
        }
        return getHeading();
    }

    public void setHeading(Rotation2d heading) {
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public void zeroAutoHeading() {
        autoPoseEstimator.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getAutoPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic() {
        poseEstimator.update(getGyroYaw(), getModulePositions());
        autoPoseEstimator.update(getGyroYaw(), getModulePositions());
        
        // Update robot orientation for all Limelights
        for (String limelightName : Constants.LimelightConstants.LIMELIGHT_NAMES) {
            LimelightHelpers.SetRobotOrientation(limelightName, getBlueHeading().getDegrees(), 0, 0, 0, 0, 0);
        }

        try {
            // Check if we should reject updates based on angular velocity
            boolean rejectDueToAngularVelocity = Math.abs(gyro.getAngularVelocityZWorld().getValueAsDouble()) > Constants.LimelightConstants.MAX_ANGULAR_VELOCITY_REJECT;

            // Process each Limelight's data
            for (String limelightName : Constants.LimelightConstants.LIMELIGHT_NAMES) {
                LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
                
                // Skip if no tags or high angular velocity
                if (rejectDueToAngularVelocity || poseEstimate.tagCount == 0) {
                    continue;
                }

                // Add vision measurement
                poseEstimator.addVisionMeasurement(
                    new Pose2d(poseEstimate.pose.getX(), poseEstimate.pose.getY(), getHeading()),
                    poseEstimate.timestampSeconds
                );

                // Log pose data to SmartDashboard
                String prefix = limelightName.substring(limelightName.lastIndexOf('-') + 1);
                SmartDashboard.putNumber(prefix + "x", poseEstimate.pose.getX());
                SmartDashboard.putNumber(prefix + "y", poseEstimate.pose.getY());
            }
        } catch (Exception e) {
            e.printStackTrace();
        }

        // Update general pose and module data
        SmartDashboard.putNumber("Gyro Yaw", getGyroYaw().getDegrees());
        SmartDashboard.putNumber("Pose X", getPose().getTranslation().getX());
        SmartDashboard.putNumber("Pose Y", getPose().getTranslation().getY());

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
    }
}