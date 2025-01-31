package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
    TalonFX leftUpMotor;
    TalonFX rightUpMotor;
    TalonFX leftDownMotor;
    TalonFX rightDownMotor;

    public TalonFXConfiguration pivotFXConfig;

    public Pivot() {
        pivotFXConfig = new TalonFXConfiguration();

        pivotFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        leftUpMotor = new TalonFX(3);
        rightUpMotor = new TalonFX(2);
        leftDownMotor = new TalonFX(4);
        rightDownMotor = new TalonFX(1);

        leftUpMotor.getConfigurator().apply(pivotFXConfig);
        rightUpMotor.getConfigurator().apply(pivotFXConfig);
        leftDownMotor.getConfigurator().apply(pivotFXConfig);
        rightDownMotor.getConfigurator().apply(pivotFXConfig);

        leftDownMotor.setControl(new Follower(leftUpMotor.getDeviceID(), false));
        rightDownMotor.setControl(new Follower(leftUpMotor.getDeviceID(), true));
        rightUpMotor.setControl(new Follower(leftUpMotor.getDeviceID(), true));
    }

    public void control(double input) {
        leftUpMotor.set(input *0.1);
    }

    @Override
    public void periodic() {
        // skip for now
    }

}
