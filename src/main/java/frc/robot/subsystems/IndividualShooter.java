package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.Amps;


public class IndividualShooter {
    private final TalonFX motor;

    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table;
    private final DoublePublisher dutyCyclePub;
    private final DoublePublisher setPointHub;
    private final DoublePublisher velocityPub;
    private final BooleanPublisher atSetpointPub;
    public final Trigger withinTolerance;
    private final double unitConversionFactor;

    public IndividualShooter(ShooterMiniConfig miniConfig, double unitConversionFactor){
        //12/20*pi*1.475/39.37
        motor = new TalonFX(miniConfig.id, "rio");
        configureMotor(miniConfig.isInverted, miniConfig.kp, miniConfig.kv, miniConfig.ks, miniConfig.ka);
        motor.getClosedLoopReference().setUpdateFrequency(200);
        this.unitConversionFactor = unitConversionFactor;
        withinTolerance = new Trigger(()-> (velocityRequest.Velocity - motor.getVelocity().getValueAsDouble())* this.unitConversionFactor < 100 * 0.05 * this.unitConversionFactor );


        table = inst.getTable("Shooter " + miniConfig.name);
        dutyCyclePub = table.getDoubleTopic("Shooter " + miniConfig.name + " Duty Cycle").publish(PubSubOption.periodic(0.02));
        setPointHub = table.getDoubleTopic("Shooter " + miniConfig.name + " Setpoint").publish(PubSubOption.periodic(0.02));
        velocityPub = table.getDoubleTopic("Shooter " + miniConfig.name + " Velocity").publish(PubSubOption.periodic(0.02));
        atSetpointPub = table.getBooleanTopic("Shooter " + miniConfig.name + " At Setpoint").publish(PubSubOption.periodic(0.02));
    }

    public void configureMotor(InvertedValue isInverted, double kp, double kv, double ks, double ka){
        TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(isInverted)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(40))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(kp)
                    .withKV(kv)
                    .withKS(ks)
                    .withKA(ka)
            );
        motor.getConfigurator().apply(config);
    }

    public void setDutyCycle(double dutyCycle){
        motor.setControl(dutyCycleRequest.withOutput(dutyCycle));
    }

    public void setVelocityRPS(double rps){
        motor.setControl(velocityRequest.withVelocity(rps));
    }

    public void sendSendables(){
        dutyCyclePub.set(motor.getDutyCycle().getValueAsDouble());
        setPointHub.set(motor.getClosedLoopReference().getValueAsDouble() * unitConversionFactor);
        velocityPub.set(motor.getVelocity().getValueAsDouble() * unitConversionFactor);
        atSetpointPub.set(withinTolerance.getAsBoolean());
    }

    public record ShooterMiniConfig(InvertedValue isInverted, double kp, double kv, double ks, double ka, String name, int id) {}
}
