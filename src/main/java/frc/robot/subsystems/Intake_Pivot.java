package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.Amps;

import java.util.function.DoubleSupplier;


public class Intake_Pivot extends SubsystemBase {
    private TalonFX pivot;

    private MotionMagicVoltage motionMagicSetter;

    private static final double kPivotReduction = 3; //make into constant, amount of rotations for arm to loop once

    private NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private NetworkTable table = inst.getTable("Intake_Pivot");

    private final DoublePublisher currentPositionDegreesPub = table.getDoubleTopic("Current Position Degrees").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher setpointPositionDegreesPub = table.getDoubleTopic("Setpoint Position Degrees").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher dutyCyclePub = table.getDoubleTopic("Duty Cycle").publish(PubSubOption.periodic(0.02));

    private final double pivotOffsetDegrees = 0;

    public Intake_Pivot() {
        pivot = new TalonFX(57);
        configurePivot();
        motionMagicSetter = new MotionMagicVoltage(0).withSlot(0);

        pivot.setPosition(pivotOffsetDegrees);
    }
    
    public void configurePivot() {
        TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
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
            );
        pivot.getConfigurator().apply(config);
    }

    public Command setDegrees(double angleDegrees) {
        return run(() -> pivot.setControl(motionMagicSetter.withPosition(angleDegrees/360.0)));
    }

    public Command setDutyCycle(DoubleSupplier velocity) {
        return run(() -> pivot.set(velocity.getAsDouble()));
    }

    public Command homePivot(){
        return setDutyCycle(() -> -0.05).until(() -> pivot.getMotorStallCurrent().getValueAsDouble() > 15).andThen(runOnce(() -> pivot.setPosition(pivotOffsetDegrees))); // make the StatorCurrentLimit (15) into a constant
    }

    @Override
    public void periodic(){
        currentPositionDegreesPub.set(pivot.getPosition().getValueAsDouble());
        setpointPositionDegreesPub.set(pivot.getClosedLoopReference().getValueAsDouble());
        dutyCyclePub.set(pivot.getDutyCycle().getValueAsDouble());
    }
}