package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(0);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Pivot s_Pivot = new Pivot();
    private final Telescope s_telescope = new Telescope();
    private final EndEffector endEffector = new EndEffector();

    /* Autos */
     private final SendableChooser<Command> autoChooser;

    // Goal position instances
    private final Constants.TargetPostioons.Coral_Goals_Left coralGoalsLeft = new Constants.TargetPostioons().new Coral_Goals_Left();
    private final Constants.TargetPostioons.Coral_Goals_Right coralGoalsRight = new Constants.TargetPostioons().new Coral_Goals_Right();
    private final Constants.TargetPostioons.Feeder_Goals feederGoals = new Constants.TargetPostioons().new Feeder_Goals();

    // take coral command
    Command takeCoral = new SequentialCommandGroup(
        new InstantCommand(() -> s_telescope.setPosition(0,false)),
        new WaitUntilCommand(s_telescope::atTargetPosition),
        new InstantCommand(() -> s_Pivot.setPosition(3.7)),
        new WaitUntilCommand(s_Pivot::atTargetPosition),
        new InstantCommand(() -> endEffector.setAngle(12)),
        new WaitUntilCommand(endEffector::atTargetPosition)
    );

    Command TallCloseRobot = new SequentialCommandGroup(
        new InstantCommand(() -> endEffector.setAngle(20)),
        new WaitUntilCommand(endEffector::atTargetPosition),
        new WaitUntilCommand(s_telescope::atTargetPosition),
        new InstantCommand(() -> s_telescope.setPosition(0,false)),
        new WaitUntilCommand(s_telescope::atTargetPosition),
        new InstantCommand(() -> s_Pivot.setPosition(0)),
        new InstantCommand(() -> endEffector.setAngle(0))
    );

    Command QuickCloseRobot = new SequentialCommandGroup(
        new InstantCommand(() -> endEffector.setAngle(5)),
        new WaitUntilCommand(endEffector::atTargetPosition),
        new WaitUntilCommand(s_telescope::atTargetPosition),
        new InstantCommand(() -> s_telescope.setPosition(0,false)),
        new WaitUntilCommand(s_telescope::atTargetPosition),
        new InstantCommand(() -> s_Pivot.setPosition(0)),
        new InstantCommand(() -> endEffector.setAngle(0))
    );

    Command VeryQuickCloseRobot = new SequentialCommandGroup(
        new InstantCommand(() -> endEffector.setAngle(5)),
        new WaitUntilCommand(endEffector::atTargetPosition),
        new WaitUntilCommand(s_telescope::atTargetPosition),
        new InstantCommand(() -> s_telescope.setPosition(0,false)),
        new WaitUntilCommand(s_telescope::atTargetPosition),
        new InstantCommand(() -> s_Pivot.setPosition(0)),
        new InstantCommand(() -> endEffector.setAngle(0))
    );

    Command coralLevel4 = new SequentialCommandGroup(
        new WaitUntilCommand(s_telescope::atTargetPosition),
        new InstantCommand(() -> s_Pivot.setPosition(11)),
        new WaitUntilCommand(s_Pivot::atTargetPosition),
        new InstantCommand(() -> endEffector.setAngle(30)),
        new WaitUntilCommand(endEffector::atTargetPosition),
        new InstantCommand(() -> s_telescope.setPosition(35,false)),
        new WaitUntilCommand(s_telescope::atTargetPosition),
        new InstantCommand(() -> endEffector.setAngle(40))
    );

    Command coralLevel3 = new SequentialCommandGroup(
        new WaitUntilCommand(s_telescope::atTargetPosition),
        new InstantCommand(() -> s_telescope.setPosition(0.6,false)),
        new InstantCommand(() -> s_Pivot.setPosition(8.6)),
        // new InstantCommand(() -> endEffector.setAngle(41)),
        new WaitUntilCommand(s_Pivot::atTargetPosition)
    );
    
    Command coralLevel2 = new SequentialCommandGroup(
        new WaitUntilCommand(s_telescope::atTargetPosition),
        new InstantCommand(() -> s_telescope.setPosition(0.6,false)),
        new InstantCommand(() -> s_Pivot.setPosition(8.6)),
        new WaitUntilCommand(s_Pivot::atTargetPosition),
        new InstantCommand(() -> endEffector.setAngle(41))
    );

    Command coralLevel1 = new SequentialCommandGroup(
        new WaitUntilCommand(s_telescope::atTargetPosition),
        new InstantCommand(() -> s_telescope.setPosition(0.6,false)),
        new InstantCommand(() -> s_Pivot.setPosition(8.6)),
        // new InstantCommand(() -> endEffector.setAngle(41)),
        new WaitUntilCommand(s_Pivot::atTargetPosition)
    );

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        // add the auto choser to the dash
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX(),
                () -> driver.leftStick().getAsBoolean(),
                () -> driver.start().getAsBoolean()
                )
        );
       
        s_telescope.setDefaultCommand(
                new TelescopeCommand(s_telescope,
                        () -> 0,
                        () -> false,
                        () -> false
                    )
        );

        endEffector.setDefaultCommand(
            new EndEffectorCommand(
                endEffector,
                () -> false,
                () -> false,
                () -> (driver.getRightTriggerAxis() - driver.getLeftTriggerAxis()) * 0.3 ,
                () -> 0
            )
        );

        s_Pivot.setDefaultCommand(
            new TeleopPivot(s_Pivot, () -> 0)
        );

        // Configure the button bindings
        configureButtonBindings();
    }   

    
    private void configureButtonBindings() {
        driver.a().onTrue(QuickCloseRobot);

        driver.y().whileTrue(coralLevel4);
        driver.y().onFalse(TallCloseRobot);
        
        driver.start().and(driver.back()).onTrue(new InstantCommand(() -> s_Swerve.zeroAutoHeading()));
        
        driver.pov(270).whileTrue(new GoTo(s_Swerve, coralGoalsLeft));
        driver.pov(270).onFalse(new StopAuto(s_Swerve));

        driver.pov(90).whileTrue(new GoTo(s_Swerve, coralGoalsRight));
        driver.pov(90).onFalse(new StopAuto(s_Swerve));

        driver.pov(180).whileTrue(new GoTo(s_Swerve, feederGoals));
        driver.pov(180).onFalse(new StopAuto(s_Swerve));
        driver.pov(180).onTrue(takeCoral);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoChooser.getSelected();
    }
}
