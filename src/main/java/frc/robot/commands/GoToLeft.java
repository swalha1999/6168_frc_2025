package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

public class GoToLeft extends Command {
    private final Swerve swerveDriveSubsystem;

    public GoToLeft(Swerve swerveDriveSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            swerveDriveSubsystem.getPose(),
            new Pose2d(14.702, 4.301, Rotation2d.fromDegrees(0))
        );

        PathConstraints constraints = new PathConstraints(1.0, 1, 2 * Math.PI, 4 * Math.PI);

        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            null,
            new GoalEndState(0.0, Rotation2d.fromDegrees(0))
        );

        path.preventFlipping = true;

        AutoBuilder.followPath(path).schedule();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
