package frc.robot.commands;

import frc.robot.Constants;

import frc.robot.subsystems.Telescope;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;


public class TelescopeCommand extends Command {    
    private Telescope telescope;    
    private DoubleSupplier translationSup;
    private BooleanSupplier overrideLimits;
    private BooleanSupplier resetPosition;
    

    public TelescopeCommand(Telescope telescope, DoubleSupplier translationSup, BooleanSupplier overrideLimits, BooleanSupplier resetPosition) {
        this.telescope = telescope;
        this.translationSup = translationSup;
        this.overrideLimits = overrideLimits;
        this.resetPosition = resetPosition;
        addRequirements(telescope);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        
        if (resetPosition.getAsBoolean()) {
            telescope.resetEncoder();
        } 
        
        telescope.adjustPosition(
            translationVal,
            overrideLimits.getAsBoolean()
        );
        
    }
}