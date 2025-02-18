package frc.robot.commands;

import frc.robot.Constants;

import frc.robot.subsystems.Telescope;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;


public class TelescopeCommand extends Command {    
    private Telescope telescope;    
    private DoubleSupplier translationSup;
    

    public TelescopeCommand(Telescope telescope, DoubleSupplier translationSup) {
        this.telescope = telescope;
        this.translationSup = translationSup;
        addRequirements(telescope);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        
        telescope.adjustPosition(
            translationVal
        );
    }
}