package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants;

public class Pivot extends SubsystemBase {
    TalonFX mainMotor;
    TalonFX rightUpMotor;
    TalonFX leftDownMotor;
    TalonFX rightDownMotor;
    
    private final PositionVoltage m_positionVoltage = new PositionVoltage(0);
    public TalonFXConfiguration pivotConfig;
    
    // Constants for PID
    private static final double kP = 1.2;
    private static final double kI = 0.1;
    private static final double kD = 0.0;
    private static final double kV = 0.12;
    
    private double targetPosition = 0.0;
    
    private final DigitalInput limitSwitch;
    private boolean pivotAtLowerLimit = false;

    public Pivot() {
        pivotConfig = new TalonFXConfiguration();

        // Configure the motor
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        // Configure PID
        pivotConfig.Slot0.kP = kP;
        pivotConfig.Slot0.kI = kI;
        pivotConfig.Slot0.kD = kD;
        pivotConfig.Slot0.kV = kV;

        pivotConfig.Voltage.PeakForwardVoltage = 12;
        pivotConfig.Voltage.PeakReverseVoltage = -12;

        mainMotor = new TalonFX(3);
        rightUpMotor = new TalonFX(2);
        leftDownMotor = new TalonFX(4);
        rightDownMotor = new TalonFX(1);

        mainMotor.getConfigurator().apply(pivotConfig);
        rightUpMotor.getConfigurator().apply(pivotConfig);
        leftDownMotor.getConfigurator().apply(pivotConfig);
        rightDownMotor.getConfigurator().apply(pivotConfig);

        leftDownMotor.setControl(new Follower(mainMotor.getDeviceID(), false));
        rightDownMotor.setControl(new Follower(mainMotor.getDeviceID(), true));
        rightUpMotor.setControl(new Follower(mainMotor.getDeviceID(), true));

        // Reset the position to 0 on startup
        mainMotor.setPosition(0);

        // Initialize limit switch on DIO port 0 (you can change this port number)
        limitSwitch = new DigitalInput(0);
    }

    /**
     * Sets the target position for the pivot
     * @param position The desired position in rotations
     */
    public void setPosition(double position) {
        targetPosition = position;
        mainMotor.setControl(m_positionVoltage.withPosition(position));
    }

    /**
     * Gets the current position of the pivot
     * @return Current position in rotations
     */
    public double getCurrentPosition() {
        return mainMotor.getPosition().getValueAsDouble();
    }

    /**
     * Checks if the pivot is at the target position
     * @return true if at target position (within tolerance)
     */
    public boolean atTargetPosition() {
        return Math.abs(getCurrentPosition() - targetPosition) < 0.1; // 0.1 rotation tolerance
    }

    /**
     * Manually adjust the position by adding to the current position
     * @param delta The amount to add to the current position (positive = up, negative = down)
     * @param overrideLimits Whether to override the limits of the pivot
     */
    public void adjustPosition(double delta, boolean overrideLimits) {
        double newPosition = targetPosition + delta;
        setPosition(newPosition);
    }

    /**
     * Gets the current target position
     * @return The target position in rotations
     */
    public double getTargetPosition() {
        return targetPosition;
    }

    /**
     * Returns whether the pivot is at its lower limit
     * @return true if the limit switch is pressed
     */
    public boolean isAtLowerLimit() {
        return !limitSwitch.get();
    }

    public void resetPivot() {
        mainMotor.setControl(new VelocityDutyCycle(0));
        targetPosition = 0;
        pivotAtLowerLimit = true;
        mainMotor.setPosition(0);
        mainMotor.setControl(m_positionVoltage.withPosition(0));
    }

    public void setPivotDownPosition() {
        targetPosition = Constants.PivotConstants.min_pivot_ticks - 0.5;
        mainMotor.setControl(m_positionVoltage.withPosition(Constants.PivotConstants.min_pivot_ticks - 0.5));
    }

    public void setPivotUpPosition() {
        targetPosition = Constants.PivotConstants.max_pivot_ticks;
        mainMotor.setControl(m_positionVoltage.withPosition(Constants.PivotConstants.max_pivot_ticks));
    }



    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Position", getCurrentPosition());
        SmartDashboard.putBoolean("Pivot Lower Limit", isAtLowerLimit());

        if (isAtLowerLimit() && !pivotAtLowerLimit) {
           resetPivot();
        }else if (!isAtLowerLimit() && pivotAtLowerLimit) {
            pivotAtLowerLimit = false;
        }

        
    }
}
