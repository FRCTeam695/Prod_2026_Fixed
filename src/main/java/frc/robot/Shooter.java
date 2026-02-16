package frc.robot;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    private final TalonFX motor;
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
    public final double kShooterRotationsToMPS = 1;
    public final double kMaxSpeedMPS = kShooterRotationsToMPS * 100;

    public Shooter(int id, InvertedValue isInverted, double kp, double kv, double ks){
        motor = new TalonFX(id, "rio");
        configureShooter(isInverted, kp, kv, ks);
    }

    public void configureShooter(InvertedValue isInverted, double kp, double kv, double ks){
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
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(kShooterRotationsToMPS)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(kp)
                    .withKV(kv)
                    .withKS(ks)
            );
        motor.getConfigurator().apply(config);
    }

    public Command setVelocityMPS(DoubleSupplier velocityMPS){
        return run(
            ()->{
                motor.setControl(velocityRequest.withVelocity(velocityMPS.getAsDouble()));
            }
        );
    }

    public Command setDutyCycle(DoubleSupplier percentVbus){
        return run(
            ()-> {
                motor.setControl(dutyCycleRequest.withOutput(percentVbus.getAsDouble()));
            }
        );
    }

    @Override
    public void initSendable(SendableBuilder builder){
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Velocity Meters Per Second", () -> motor.getVelocity().getValueAsDouble(), null);
        builder.addDoubleProperty("Stator Current", () -> motor.getStatorCurrent().getValue().in(Amps), null);
        builder.addDoubleProperty("Supply Current", () -> motor.getSupplyCurrent().getValue().in(Amps), null);
        builder.addDoubleProperty("Velocity Referance MPS", ()-> motor.getClosedLoopReference().getValueAsDouble(), null);
    }

}
