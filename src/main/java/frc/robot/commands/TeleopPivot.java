package frc.robot.commands;

import frc.robot.subsystems.Pivot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopPivot extends Command {    
    private Pivot s_Pivot;    
    private DoubleSupplier translationVal;

    public TeleopPivot(Pivot s_Pivot, DoubleSupplier translationVal) {
        this.s_Pivot = s_Pivot;
        this.translationVal = translationVal;
        addRequirements(s_Pivot);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        
        s_Pivot.adjustPosition(
            MathUtil.applyDeadband(translationVal.getAsDouble(), 0.1), 
            false
        );

        SmartDashboard.putNumber("the pivot command", translationVal.getAsDouble());
    }
}