package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int right_bumber = XboxController.Axis.kRightTrigger.value;
    private final int left_bumber = XboxController.Axis.kLeftTrigger.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    // private final int Pivotangle = XboxController.Axis.kRightY.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);
    
    private final JoystickButton driverY = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton driverA = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton driverX = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton driverB = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton driverLeftBumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton driverLeftStick = new JoystickButton(driver, XboxController.Button.kLeftStick.value);
    private final JoystickButton driverRightBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Pivot s_Pivot = new Pivot();
    private final Telescope s_telescope = new Telescope();
    private final EndEffector endEffector = new EndEffector();

    /* Autos */
     private final SendableChooser<Command> autoChooser;

    // take coral command
    Command takeCoral = new SequentialCommandGroup(
        new InstantCommand(() -> s_telescope.setPosition(0,false)),
        new WaitUntilCommand(s_telescope::atTargetPosition),
        new InstantCommand(() -> s_Pivot.setPosition(3.6)),
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
        new InstantCommand(() -> s_telescope.setPosition(60,false)),
        new WaitUntilCommand(s_telescope::atTargetPosition),
        new InstantCommand(() -> endEffector.setAngle(41.8))
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
                () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis),
                () -> -driver.getRawAxis(rotationAxis),
                () -> driverLeftStick.getAsBoolean(),
                () -> zeroGyro.getAsBoolean()
                )
        );
       
        s_telescope.setDefaultCommand(
                new TelescopeCommand(s_telescope,
                        () -> 0,
                        () -> false,
                        () -> zeroGyro.getAsBoolean()
                    )
        );

        endEffector.setDefaultCommand(
            new EndEffectorCommand(
                endEffector,
                () -> false,
                () -> false,
                () -> (driver.getRawAxis(right_bumber) - driver.getRawAxis(left_bumber)) * 0.5 ,
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
        driverX.onTrue(takeCoral);
        driverA.onTrue(QuickCloseRobot);

        driverY.whileTrue(coralLevel4);
        driverY.onFalse(TallCloseRobot);
        
        driverB.whileTrue(new GoToLeft(s_Swerve));
        driverB.onFalse(new StopAuto(s_Swerve));
        

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
