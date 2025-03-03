package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;

public class EndEffectorCommand extends Command {
    private final EndEffector endEffector;
    private final BooleanSupplier angleUp;
    private final BooleanSupplier angleDown;
    private final DoubleSupplier intakeSignle;
    private final DoubleSupplier endEffectorAngle;

    /**
     * Creates a new EndEffectorCommand.
     * 
     * @param endEffector The EndEffector subsystem
     * @param angleSupplier Supplier for the target angle (typically from a joystick)
     * @param intakeSupplier Supplier for the intake speed (typically from a trigger or button)
     */
    public EndEffectorCommand(
        EndEffector endEffector, 
        BooleanSupplier angleUp,
        BooleanSupplier angleDown,
        DoubleSupplier intakeSignle,
        DoubleSupplier endEffectorAngle
    ) {
        this.endEffector = endEffector;
        this.angleUp = angleUp;
        this.angleDown = angleDown;
        this.intakeSignle = intakeSignle;
        this.endEffectorAngle = endEffectorAngle; 
        addRequirements(endEffector);
    }

    @Override
    public void execute() {
        // Update the target angle based on joystick input
        double angle_speed = 0;
        if (angleUp.getAsBoolean()) {
            angle_speed+=0.1;
        } else if (angleDown.getAsBoolean()) {
            angle_speed-=0.1;
        }
        angle_speed += MathUtil.applyDeadband(endEffectorAngle.getAsDouble(), Constants.stickDeadband);
        endEffector.adjustAngle(angle_speed);
        
        // Update intake speed
        endEffector.setIntakeSpeed(MathUtil.applyDeadband(intakeSignle.getAsDouble(), Constants.stickDeadband));
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the intake when the command ends
        endEffector.setIntakeSpeed(0);
    }

    @Override
    public boolean isFinished() {
        // This is a continuous command that runs until interrupted
        return false;
    }
} 