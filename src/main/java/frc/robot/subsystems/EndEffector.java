package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;



import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffector extends SubsystemBase {
    private final SparkFlex angleMotor;
    private final SparkFlex intakeMotor;
    
    private final RelativeEncoder  angleEncoder;
    private final SparkClosedLoopController closedLoopController;

    private final SparkFlexConfig angleConfig = new SparkFlexConfig();
    private final SparkFlexConfig intakeConfig = new SparkFlexConfig();
    
    private double targetAngle = 0.0;

    public EndEffector() {
        // Initialize motors
        angleMotor = new SparkFlex(EndEffectorConstants.ANGLE_MOTOR_ID, MotorType.kBrushless);
        intakeMotor = new SparkFlex(EndEffectorConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
        
        // Get encoder from angle motor
        angleEncoder = angleMotor.getEncoder();
        closedLoopController = angleMotor.getClosedLoopController();

        angleConfig.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);
        

        angleConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(-0.25, 0.25)
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-0.2, 0.2, ClosedLoopSlot.kSlot1);

        intakeConfig.idleMode(IdleMode.kBrake);
        angleConfig.idleMode(IdleMode.kBrake);

        angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        
        // Set motor inversions if needed
        // angleMotor.setInverted(false);
        // intakeMotor.setInverted(false);
    }
    
    @Override
    public void periodic() {
        
        // Output telemetry
        SmartDashboard.putNumber("End Effector Angle", angleEncoder.getPosition());
        SmartDashboard.putNumber("End Effector Target", targetAngle);
        
    }
    
    public void adjustAngle(double angle) {
        targetAngle += angle;
        targetAngle = MathUtil.clamp(targetAngle, EndEffectorConstants.MIN_ANGLE_POSITION, EndEffectorConstants.MAX_ANGLE_POSITION);
        closedLoopController.setReference(targetAngle, ControlType.kPosition);
    }

    public void setAngle(double angle) {
        targetAngle = angle;
        closedLoopController.setReference(targetAngle, ControlType.kPosition);
    }
    
    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }
    
} 