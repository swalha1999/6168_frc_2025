package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Swerve;

public class StopAuto extends Command {
    private final Swerve swerveDriveSubsystem;

    public StopAuto(Swerve swerveDriveSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        // remove interupt all the command that need the sweve
        CommandScheduler.getInstance().cancel(swerveDriveSubsystem.getCurrentCommand());
    }

    @Override
    public boolean isFinished() {
        return true;
    }


}
