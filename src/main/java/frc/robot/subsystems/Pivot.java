package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
    TalonFX leftUpMotor;
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

    public Pivot() {
        pivotConfig = new TalonFXConfiguration();

        // Configure the motor
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Configure PID
        pivotConfig.Slot0.kP = kP;
        pivotConfig.Slot0.kI = kI;
        pivotConfig.Slot0.kD = kD;
        pivotConfig.Slot0.kV = kV;

        pivotConfig.Voltage.PeakForwardVoltage = 12;
        pivotConfig.Voltage.PeakReverseVoltage = -12;

        leftUpMotor = new TalonFX(3);
        rightUpMotor = new TalonFX(2);
        leftDownMotor = new TalonFX(4);
        rightDownMotor = new TalonFX(1);

        leftUpMotor.getConfigurator().apply(pivotConfig);
        rightUpMotor.getConfigurator().apply(pivotConfig);
        leftDownMotor.getConfigurator().apply(pivotConfig);
        rightDownMotor.getConfigurator().apply(pivotConfig);

        leftDownMotor.setControl(new Follower(leftUpMotor.getDeviceID(), false));
        rightDownMotor.setControl(new Follower(leftUpMotor.getDeviceID(), true));
        rightUpMotor.setControl(new Follower(leftUpMotor.getDeviceID(), true));

        // Reset the position to 0 on startup
        leftUpMotor.setPosition(0);
    }

    /**
     * Sets the target position for the pivot
     * @param position The desired position in rotations
     */
    public void setPosition(double position) {
        targetPosition = position;
        leftUpMotor.setControl(m_positionVoltage.withPosition(position));
    }

    /**
     * Gets the current position of the pivot
     * @return Current position in rotations
     */
    public double getCurrentPosition() {
        return leftUpMotor.getPosition().getValueAsDouble();
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
     */
    public void adjustPosition(double delta) {
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

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
