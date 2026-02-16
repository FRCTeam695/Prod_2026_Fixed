package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import static edu.wpi.first.units.Units.Amps;


public class IntakeRoller extends SubsystemBase {
    private TalonFX roller;

    private NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private NetworkTable table = inst.getTable("Intake_Roller");

    private final DoublePublisher currentVelocityMPSPub = table.getDoubleTopic("Current Velocity MPS").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher setpointVelocityMPSPub = table.getDoubleTopic("Setpoint Velocity MPS").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher dutyCyclePub = table.getDoubleTopic("Duty Cycle").publish(PubSubOption.periodic(0.02));


    private final VelocityVoltage velocity = new VelocityVoltage(0);

    public final double MAX_RPS = 100;

    public IntakeRoller() {
        roller = new TalonFX(53);
        configureRoller();
    }

    public void configureRoller() {
        TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
               new MotorOutputConfigs()
               .withInverted(InvertedValue.Clockwise_Positive)
               .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(15))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(15))
                .withSupplyCurrentLimitEnable(true)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(0)
                    .withKI(0)
                    .withKD(0)
                    .withKV(0.12)
                    .withKS(0.18)
            );
        roller.getConfigurator().apply(config);
    }

    public Command setVelocityRPS(DoubleSupplier targetVelocityRPS) {
        return run(() -> roller.setControl(velocity.withVelocity(targetVelocityRPS.getAsDouble())));
    }

    public Command setDutyCycle(DoubleSupplier percent) {
        return run(() -> roller.set(percent.getAsDouble()));
    }

    @Override
    public void periodic(){
        currentVelocityMPSPub.set(roller.getVelocity().getValueAsDouble());
        setpointVelocityMPSPub.set(roller.getClosedLoopReference().getValueAsDouble());
        dutyCyclePub.set(roller.getDutyCycle().getValueAsDouble());
    }
}