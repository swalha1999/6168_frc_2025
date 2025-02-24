package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.util.ArmUtil;
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
    private final Joystick debug =  new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int right_bumber = XboxController.Axis.kRightTrigger.value;
    private final int left_bumber = XboxController.Axis.kLeftTrigger.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    private final int Pivotangle = XboxController.Axis.kRightY.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    
    private final JoystickButton driverY = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton driverA = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton driverX = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton driverB = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton driverLeftBumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton driverRightBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    
    /* Debug Controls*/
    private final JoystickButton overrideLimits = new JoystickButton(debug, XboxController.Button.kBack.value);
    private final JoystickButton endEffectorUp = new JoystickButton(debug, XboxController.Button.kY.value);
    
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Pivot s_Pivot = new Pivot();
    private final Telescope s_telescope = new Telescope();
    private final EndEffector endEffector = new EndEffector();

    double[] armPose = ArmUtil.solveEA(82, 82, 20, 38);

    // take coral command
    Command takeCoral = new SequentialCommandGroup(
        new InstantCommand(() -> s_telescope.setPosition(1,false)),
        new WaitUntilCommand(s_telescope::atTargetPosition),
        new InstantCommand(() -> s_Pivot.setPosition(4.1)),
        new WaitUntilCommand(s_Pivot::atTargetPosition),
        new InstantCommand(() -> endEffector.setAngle(12)),
        new InstantCommand(() -> s_telescope.setPosition(7.22,false))
        
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
    
    Command coralLevel1 = new SequentialCommandGroup(
        new InstantCommand(() -> s_telescope.setPosition(0.6,false)),
        new InstantCommand(() -> s_Pivot.setPosition(8.6)),
        new InstantCommand(() -> endEffector.setAngle(41)),
        new WaitUntilCommand(s_Pivot::atTargetPosition)
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
                () -> robotCentric.getAsBoolean(),
                () -> zeroGyro.getAsBoolean()
                )
        );

        driverX.whileTrue(takeCoral).onFalse(closeRobot);
        driverY.whileTrue(coralLevel4).onFalse(closeRobot);
        driverA.whileTrue(coralLevel1).onFalse(closeRobot);
       
        s_telescope.setDefaultCommand(
                new TelescopeCommand(s_telescope,
                        () -> debug.getRawAxis(right_bumber) - debug.getRawAxis(left_bumber),
                        () -> overrideLimits.getAsBoolean(),
                        () -> zeroGyro.getAsBoolean()
                    )
        );

        endEffector.setDefaultCommand(
            new EndEffectorCommand(
                endEffector,
                () -> false,
                () -> false,
                () -> driverLeftBumper.getAsBoolean(),
                () -> driverRightBumper.getAsBoolean(),
                () -> debug.getRawAxis(translationAxis)
            )
        );

        s_Pivot.setDefaultCommand(
            new TeleopPivot(s_Pivot, () -> debug.getRawAxis(Pivotangle))
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
