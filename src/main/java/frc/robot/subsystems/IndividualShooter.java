package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
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
    private final VelocityTorqueCurrentFOC torqueRequest = new VelocityTorqueCurrentFOC(0.0);

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table;
    private final DoublePublisher dutyCyclePub;
    private final DoublePublisher setPointHub;
    private final DoublePublisher velocityPub;
    private final BooleanPublisher atSetpointPub;
    private final BooleanPublisher stoppedShootingPub;
    public final Trigger withinTolerance;
    private final double unitConversionFactor;
    private final ShooterMiniConfig myConfig;
    private double setpoint;

    private final Debouncer shooterDipDebouncer;

    private final double dipThreshold = 0.0;

    private boolean hasSeenDip = false;

    public IndividualShooter(ShooterMiniConfig miniConfig, double unitConversionFactor){
        //12/20*pi*1.475/39.37
        motor = new TalonFX(miniConfig.id, "rio");
        myConfig = miniConfig;
        //configForBangBang();
        configForVelocityControl();
        motor.getClosedLoopReference().setUpdateFrequency(200);

        this.unitConversionFactor = unitConversionFactor;

        // within 5% tolerance
        withinTolerance = new Trigger(()-> Math.abs(setpoint - motor.getVelocity().getValueAsDouble()) < 5);

        table = inst.getTable("Shooter " + miniConfig.name);
        dutyCyclePub = table.getDoubleTopic("Shooter " + miniConfig.name + " Duty Cycle").publish(PubSubOption.periodic(0.02));
        setPointHub = table.getDoubleTopic("Shooter " + miniConfig.name + " Setpoint").publish(PubSubOption.periodic(0.02));
        velocityPub = table.getDoubleTopic("Shooter " + miniConfig.name + " Velocity").publish(PubSubOption.periodic(0.02));
        atSetpointPub = table.getBooleanTopic("Shooter " + miniConfig.name + " At Setpoint").publish(PubSubOption.periodic(0.02));
        stoppedShootingPub = table.getBooleanTopic("Shooter " + miniConfig.name + " Stopped Shooting").publish(PubSubOption.periodic(0.02));

        shooterDipDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);
    }

    public void configForVelocityControl(){
        TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(myConfig.isInverted)
                    .withNeutralMode(NeutralModeValue.Coast)
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
                    .withKP(myConfig.kp)
                    .withKV(myConfig.kv)
                    .withKS(myConfig.ks)
                    .withKA(myConfig.ka)
            );
        motor.getConfigurator().apply(config);
    }

    public void setDutyCycle(double dutyCycle){
        motor.setControl(dutyCycleRequest.withOutput(dutyCycle));
    }

    public void setVelocityRPS(double rps){
        hasSeenDip = false;
        setpoint = rps;
        motor.setControl(velocityRequest.withVelocity(rps));
    }

    public void setTorqueCurrent(double rps){
        hasSeenDip = false;
        setpoint = rps;
        motor.setControl(torqueRequest.withVelocity(rps));
    }

    public void configForBangBang(){
        TalonFXConfiguration config = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(40))
                    .withSupplyCurrentLimitEnable(true)
            ).withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(myConfig.isInverted)
                    .withNeutralMode(NeutralModeValue.Coast)
            );
        config.Slot0.kP = 999999.0;
        config.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
        config.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;
        config.MotorOutput.PeakForwardDutyCycle = 1.0;
        config.MotorOutput.PeakReverseDutyCycle = 0.0;

        motor.getConfigurator().apply(config);
    }

    public double getMotorVelocityMPS(){
        return motor.getVelocity().getValueAsDouble() * unitConversionFactor;
    }

    public boolean stoppedShooting(){

        double velocity = getMotorVelocityMPS();
        double setpoint = motor.getClosedLoopReference().getValueAsDouble() * unitConversionFactor;

        boolean shooterDip = setpoint - velocity > dipThreshold;
        
        if (shooterDip) { 
            // meant to prevent it from going off before a shot has happened
            hasSeenDip = true;
        }

        return hasSeenDip && shooterDipDebouncer.calculate(!shooterDip);
    }

    public void sendSendables(){
        dutyCyclePub.set(motor.getDutyCycle().getValueAsDouble());
        setPointHub.set(motor.getClosedLoopReference().getValueAsDouble() * unitConversionFactor);
        velocityPub.set(motor.getVelocity().getValueAsDouble() * unitConversionFactor);
        atSetpointPub.set(withinTolerance.getAsBoolean());
        stoppedShootingPub.set(stoppedShooting());
    }

    public record ShooterMiniConfig(InvertedValue isInverted, double kp, double kv, double ks, double ka, String name, int id) {}
}
