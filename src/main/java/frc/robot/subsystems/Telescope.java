package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    private double targetPosition = 0.0;

    public Telescope() {
        telescopeConfig = new TalonFXConfiguration();

        // Configure the motor
        telescopeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Set slot 0 gains
        var slot0Configs = telescopeConfig.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        // Set Motion Magic settings
        var motionMagicConfigs = telescopeConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 120; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 90; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

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
        setMotionMagicPosition(position);
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
        return Math.abs(getCurrentPosition() - targetPosition) < 3; // 0.1 rotation tolerance
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

    /**
     * Creates a Motion Magic request and sets the target position to 100 rotations
     */
    public void setMotionMagicPosition(double position) {   
        mainMotor.setControl(m_request.withPosition(position));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Telescope Position", getCurrentPosition());
        SmartDashboard.putNumber("Telescope Target Position", getTargetPosition());
    }
}
