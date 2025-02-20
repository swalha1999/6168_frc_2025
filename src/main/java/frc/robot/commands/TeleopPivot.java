package frc.robot.commands;

import frc.robot.subsystems.Pivot;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopPivot extends Command {    
    private Pivot s_Pivot;    
    private BooleanSupplier up;
    private BooleanSupplier down;
    

    public TeleopPivot(Pivot s_Pivot, BooleanSupplier up, BooleanSupplier down) {
        this.s_Pivot = s_Pivot;
        this.up = up;
        this.down = down;
        addRequirements(s_Pivot);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = 0;
        
        if (up.getAsBoolean()){
            translationVal+=0.1;
            // s_Pivot.setPivotUpPosition();
        }
        if(down.getAsBoolean() && !s_Pivot.isAtLowerLimit()){
            translationVal-=0.1;
            // s_Pivot.setPivotDownPosition();
        }
        
        s_Pivot.adjustPosition(
            translationVal, 
            false
        );

        SmartDashboard.putNumber("the pivot command", translationVal);
    }
}