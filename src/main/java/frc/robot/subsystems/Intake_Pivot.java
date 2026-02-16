package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.Amps;

import java.util.function.DoubleSupplier;


public class Intake_Pivot extends SubsystemBase {
    private TalonFX pivot;

    private MotionMagicVoltage motionMagicSetter;
    private VoltageOut voltageSetter;

    private static final double kPivotReduction = 46.4; //make into constant, amount of rotations for arm to loop once

    private NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private NetworkTable table = inst.getTable("Intake_Pivot");

    private final DoublePublisher currentPositionRotationsPub = table.getDoubleTopic("Current Position Rotations").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher setpointPositionRotationsPub = table.getDoubleTopic("Setpoint Position Rotations").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher dutyCyclePub = table.getDoubleTopic("Duty Cycle").publish(PubSubOption.periodic(0.02));

    private final double pivotRetractedPositionRotations = 0.056;
    private final double pivotExtendedPositionRotations = -0.239; 

    public Intake_Pivot() {

        // as intake moves from retracted to extended, rotations will become negative

        pivot = new TalonFX(57);
        configurePivot();
        motionMagicSetter = new MotionMagicVoltage(0).withSlot(0);
        voltageSetter = new VoltageOut(0);

        pivot.setPosition(pivotRetractedPositionRotations);
    }
    
    public void configurePivot() {
        TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withCurrentLimits( //taken from WCP code
                new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(15))//was 120
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(15)) //was 70
                .withSupplyCurrentLimitEnable(true)
            )
            .withFeedback(
                new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                .withSensorToMechanismRatio(kPivotReduction)
            )
            .withMotionMagic(
                new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(90) //NEEDS TO BE CHANGED, copied from Goldfish elevator
                .withMotionMagicAcceleration(500) //NEEDS TO BE CHANGED, copied from Goldfish elevator
            )
            .withSlot0(
                new Slot0Configs()
                .withKP(0) //NEED TO CHANGE
                .withKI(0) //NEED TO CHANGE (add kd and kv if needed)
                .withKV(0)
                .withKG(0)
                .withKS(0)

                // kg + ks = 0.28 V

            ).withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(pivotRetractedPositionRotations)
                .withReverseSoftLimitThreshold(pivotExtendedPositionRotations)
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitEnable(true)
            );
        pivot.getConfigurator().apply(config);

    }

    public Command setDegrees(double angleDegrees) {
        return run(() -> pivot.setControl(motionMagicSetter.withPosition(angleDegrees/360.0)));
    }

    public Command setDutyCycle(DoubleSupplier velocity) {
        return run(() -> pivot.set(velocity.getAsDouble()));
    }

    public Command setVoltage(DoubleSupplier voltage){
        return run(() -> pivot.setControl(voltageSetter.withOutput(voltage.getAsDouble())));
    }

    public Command homePivot(){
        return setDutyCycle(() -> -0.05).until(() -> pivot.getMotorStallCurrent().getValueAsDouble() > 15).andThen(runOnce(() -> pivot.setPosition(pivotRetractedPositionRotations))); // make the StatorCurrentLimit (15) into a constant
    }

   

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Voltage", pivot.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Current", pivot.getStatorCurrent().getValueAsDouble());


        currentPositionRotationsPub.set(pivot.getPosition().getValueAsDouble());
        setpointPositionRotationsPub.set(pivot.getClosedLoopReference().getValueAsDouble());
        dutyCyclePub.set(pivot.getDutyCycle().getValueAsDouble());
    }
}