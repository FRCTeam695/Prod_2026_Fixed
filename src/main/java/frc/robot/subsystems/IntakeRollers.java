package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import static edu.wpi.first.units.Units.Amps;


public class IntakeRollers extends SubsystemBase {
    private TalonFX leaderRoller;
    private TalonFX followerRoller;

    private NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private NetworkTable table = inst.getTable("Intake_Roller");

    private final DoublePublisher currentVelocityMPSPub = table.getDoubleTopic("Current Velocity RPS").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher setpointVelocityMPSPub = table.getDoubleTopic("Setpoint Velocity RPS").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher dutyCyclePub = table.getDoubleTopic("Duty Cycle").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher tempPub = table.getDoubleTopic("Temperature").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher voltagePub = table.getDoubleTopic("Applied Voltage").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher statorCurrentPub = table.getDoubleTopic("Stator Current").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher supplyCurrentPub = table.getDoubleTopic("Supply Current").publish(PubSubOption.periodic(0.02));


    private final int leader_ID = 53;
    private final VelocityVoltage velocity = new VelocityVoltage(0);
    private final Follower followRequest = new Follower(leader_ID, MotorAlignmentValue.Opposed);
    public final double kMaxVelocity = 100;
    private final VelocityTorqueCurrentFOC torqueRequest = new VelocityTorqueCurrentFOC(0);

    public IntakeRollers() {
        leaderRoller = new TalonFX(leader_ID);
        followerRoller = new TalonFX(60);
        configureRollers();
        followerRoller.setControl(followRequest);
    }

    public void configureRollers() {
        TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
               new MotorOutputConfigs()
               .withInverted(InvertedValue.Clockwise_Positive)
               .withNeutralMode(NeutralModeValue.Coast)
                .withPeakForwardDutyCycle(1)
                .withPeakReverseDutyCycle(-1)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(60)) //80
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(40)) //60
                .withSupplyCurrentLimitEnable(true)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(0.3)
                    // .withKI(0)
                    // .withKD(0)
                    .withKV(0.12)
                    .withKS(0.18)
            );
            
        leaderRoller.getConfigurator().apply(config);
        followerRoller.getConfigurator().apply(config);

        followerRoller.getDeviceTemp().setUpdateFrequency(8);
        leaderRoller.getDeviceTemp().setUpdateFrequency(8);
    }

    public Command setVelocityRPS(DoubleSupplier targetVelocityRPS) {
        return run(() -> leaderRoller.setControl(velocity.withVelocity(targetVelocityRPS.getAsDouble())));
    }

    public Command setDutyCycle(DoubleSupplier percent) {
        return run(() -> leaderRoller.set(percent.getAsDouble()));
    }

    public Command setTorqueCurrent(DoubleSupplier rps){
        return run(()->leaderRoller.setControl(torqueRequest.withVelocity(rps.getAsDouble())));
    }

    @Override
    public void periodic(){
        currentVelocityMPSPub.set(leaderRoller.getVelocity().getValueAsDouble());
        setpointVelocityMPSPub.set(leaderRoller.getClosedLoopReference().getValueAsDouble());
        dutyCyclePub.set(leaderRoller.getDutyCycle().getValueAsDouble());
        tempPub.set(leaderRoller.getDeviceTemp().getValueAsDouble());
        voltagePub.set(leaderRoller.getMotorVoltage().getValueAsDouble());
        statorCurrentPub.set(leaderRoller.getStatorCurrent().getValueAsDouble());
        supplyCurrentPub.set(leaderRoller.getSupplyCurrent().getValueAsDouble());
    }
}