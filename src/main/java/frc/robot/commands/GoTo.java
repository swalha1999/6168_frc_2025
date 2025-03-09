package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.io.Console;
import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import frc.robot.Constants;
import frc.robot.Constants.TargetPostioons.Goals;

public class GoTo extends Command {
    private final Swerve swerveDriveSubsystem;
    Goals TargetGoals;
    private Pose2d currentGoal;

    public GoTo(Swerve swerveDriveSubsystem, Constants.TargetPostioons.Goals goals) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.TargetGoals = goals;
        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        try {
            currentGoal = getGoal(TargetGoals);

            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                    swerveDriveSubsystem.getAutoPose(),
                    currentGoal);

            PathConstraints constraints = new PathConstraints(1.0, 1, 3 * Math.PI, 6 * Math.PI);

            PathPlannerPath path = new PathPlannerPath(
                    waypoints,
                    constraints,
                    null,
                    new GoalEndState(0.0, currentGoal.getRotation()));

            path.preventFlipping = true;

            System.out.println("Going to: " + currentGoal);

            AutoBuilder.followPath(path).schedule();
        } catch (Exception e) {
            System.out.println("Going to: " + currentGoal);
            System.out.println("Falid to generate path");
        }

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }

    public Pose2d getGoal(Goals goals) {
        Pose2d[] goalsArray = goals.Blue;

        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                goalsArray = goals.Red;
            }
        }
        
        
        Pose2d currentPose = swerveDriveSubsystem.getAutoPose();
        double minDistance = Double.MAX_VALUE;
        Pose2d closestGoal = null;



        for (Pose2d goal : goalsArray) {
            double distance = currentPose.getTranslation().getDistance(goal.getTranslation());
            if (distance < minDistance) {
                minDistance = distance;
                closestGoal = goal;
            }
        }
        return closestGoal;
    }
}
