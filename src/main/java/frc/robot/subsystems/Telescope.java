package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Telescope extends SubsystemBase {
    TalonFX firstMotor;
    TalonFX secondUpMotor;

    private final PositionVoltage m_positionVoltage = new PositionVoltage(0);
    public TalonFXConfiguration telescopeConfig;

    // Constants for PID
    private static final double kP = 1.2;
    private static final double kI = 0.1;
    private static final double kD = 0.0;
    private static final double kV = 0.12;

    private double targetPosition = 0.0;

    public Telescope() {
        telescopeConfig = new TalonFXConfiguration();

        // Configure the motor
        telescopeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Configure PID
        telescopeConfig.Slot0.kP = kP;
        telescopeConfig.Slot0.kI = kI;
        telescopeConfig.Slot0.kD = kD;
        telescopeConfig.Slot0.kV = kV;

        telescopeConfig.Voltage.PeakForwardVoltage = 12;
        telescopeConfig.Voltage.PeakReverseVoltage = -12;

        firstMotor = new TalonFX(0);
        secondUpMotor = new TalonFX(6);

        firstMotor.getConfigurator().apply(telescopeConfig);
        secondUpMotor.getConfigurator().apply(telescopeConfig);
        
        secondUpMotor.setControl(new Follower(firstMotor.getDeviceID(), false));
        
        // Reset the position to 0 on startup
        firstMotor.setPosition(0);
    }

    /**
     * Sets the target position for the telescope
     * @param position The desired position in rotations
     */
    public void setPosition(double position) {
        targetPosition = position;
        firstMotor.setControl(m_positionVoltage.withPosition(position));
    }

    /**
     * Gets the current position of the telescope
     * @return Current position in rotations
     */
    public double getCurrentPosition() {
        return firstMotor.getPosition().getValueAsDouble();
    }

    /**
     * Checks if the telescope is at the target position
     * @return true if at target position (within tolerance)
     */
    public boolean atTargetPosition() {
        return Math.abs(getCurrentPosition() - targetPosition) < 0.1; // 0.1 rotation tolerance
    }

    /**
     * Manually adjust the position by adding to the current position
     * @param delta The amount to add to the current position (positive = extend, negative = retract)
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
