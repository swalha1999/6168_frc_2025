package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Pivot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopPivot extends Command {    
    private Pivot s_Pivot;    
    private DoubleSupplier translationSup;
    

    public TeleopPivot(Pivot s_Pivot, DoubleSupplier translationSup) {
        this.s_Pivot = s_Pivot;
        this.translationSup = translationSup;
        addRequirements(s_Pivot);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        
        s_Pivot.control(
            translationVal
        );
    }
}