package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.autos.*;
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

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);
    // private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    // private final JoystickButton resetTelescope = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton overrideLimits = new JoystickButton(driver, XboxController.Button.kBack.value);

    private final JoystickButton pivotUp = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton pivotDown = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton endEffectorUp = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton endEffectorDown = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton intakeIn = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton intakeOut = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Pivot s_Pivot = new Pivot();
    private final Telescope s_telescope = new Telescope();
    private final EndEffector endEffector = new EndEffector();


    // take coral command
    Command takeCoral = new SequentialCommandGroup(
        new InstantCommand(() -> s_telescope.setPosition(1,false)),
        new WaitUntilCommand(s_telescope::atTargetPosition),
        new InstantCommand(() -> s_Pivot.setPosition(4)),
        new InstantCommand(() -> endEffector.setAngle(9))
    );

    Command closeRobot = new SequentialCommandGroup(
        new InstantCommand(() -> s_telescope.setPosition(1,false)),
        new WaitUntilCommand(s_telescope::atTargetPosition),
        new InstantCommand(() -> s_Pivot.setPosition(0)),
        new InstantCommand(() -> endEffector.setAngle(0))
    );

    Command coralLevel4 = new SequentialCommandGroup(
        new InstantCommand(() -> s_Pivot.setPosition(10)),
        new InstantCommand(() -> endEffector.setAngle(40)),
        new WaitUntilCommand(s_Pivot::atTargetPosition),
        new InstantCommand(() -> s_telescope.setPosition(70,false))
    ); 


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
        new TeleopSwerve(
        s_Swerve,
        () -> driver.getRawAxis(translationAxis),
        () -> driver.getRawAxis(strafeAxis),
        () -> driver.getRawAxis(rotationAxis),
        () -> false,
        () -> overrideLimits.getAsBoolean()
        )
        );

        endEffectorUp.onTrue(takeCoral);
        pivotDown.onTrue(closeRobot);
        pivotUp.onTrue(coralLevel4);
       
        s_telescope.setDefaultCommand(
                new TelescopeCommand(s_telescope,
                        () -> driver.getRawAxis(right_bumber) - driver.getRawAxis(left_bumber),
                        () -> overrideLimits.getAsBoolean(),
                        () -> zeroGyro.getAsBoolean()
                    )
        );

        endEffector.setDefaultCommand(
            new EndEffectorCommand(
                endEffector,
                () -> endEffectorUp.getAsBoolean(),
                () -> endEffectorDown.getAsBoolean(),
                () -> intakeIn.getAsBoolean(),
                () -> intakeOut.getAsBoolean()
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }   

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        // zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}
