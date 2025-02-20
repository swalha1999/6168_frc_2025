package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Telescope extends SubsystemBase {
    TalonFX mainMotor;
    TalonFX slaveMotor;

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

        mainMotor = new TalonFX(0);
        slaveMotor = new TalonFX(6);

        mainMotor.getConfigurator().apply(telescopeConfig);
        slaveMotor.getConfigurator().apply(telescopeConfig);
        
        slaveMotor.setControl(new Follower(mainMotor.getDeviceID(), false));
        
        // Reset the position to 0 on startup
        mainMotor.setPosition(0);
    }

    /**
     * Sets the target position for the telescope
     * @param position The desired position in rotations
     */
    public void setPosition(double position, boolean overrideLimits) {
        if (!overrideLimits){
            if (position > Constants.TelescopeConstants.max_extension_ticks) {
                position = Constants.TelescopeConstants.max_extension_ticks;
            } else if (position < Constants.TelescopeConstants.min_extension_ticks) {
                position = Constants.TelescopeConstants.min_extension_ticks;
            }
        }
        targetPosition = position;
        mainMotor.setControl(m_positionVoltage.withPosition(targetPosition));
    }

    /**
     * Resets the encoder to make the current position the new zero point
     */
    public void resetEncoder() {
        mainMotor.setControl(new VelocityDutyCycle(0));
        targetPosition = 0;
        mainMotor.setPosition(0);
        mainMotor.setControl(m_positionVoltage.withPosition(0));
    }

    /**
     * Gets the current position of the telescope
     * @return Current position in rotations
     */
    public double getCurrentPosition() {
        return mainMotor.getPosition().getValueAsDouble();
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
    public void adjustPosition(double delta, boolean overrideLimits) {
        double newPosition = targetPosition + delta;
        setPosition(newPosition, overrideLimits);
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
        SmartDashboard.putNumber("Telescope Position", getCurrentPosition());
        SmartDashboard.putNumber("Telescope Target Position", getTargetPosition());
    }
}
