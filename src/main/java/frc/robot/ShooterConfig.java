package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import java.io.ObjectInputFilter.Config;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ShooterConfig extends SubsystemBase{

    private TalonFX talonFX;
    private TalonFXConfiguration configs;

    private VelocityDutyCycle dutyRequest;
    private VelocityTorqueCurrentFOC torqueCurrentRequest;
    private VelocityVoltage voltageRequest;

    public ShooterConfig(int ID) {
        // Talon controls
        talonFX = new TalonFX(15);
        configs = new TalonFXConfiguration();

        // bangbang control types
        dutyRequest = new VelocityDutyCycle(0);
        torqueCurrentRequest = new VelocityTorqueCurrentFOC(0);
        voltageRequest = new VelocityVoltage(0);

        // Limits and modes 
        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        configs.CurrentLimits.SupplyCurrentLimitEnable = true;
        configs.CurrentLimits.SupplyCurrentLimit = 40;

        configs.Slot0.kP = 999999.0;
        
        // Bang bang voltage use in startup and recovery
        // Bang bang current-torque control in idle and ball mode
        configs.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
        configs.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;
        configs.MotorOutput.PeakForwardDutyCycle = 1.0;
        configs.MotorOutput.PeakReverseDutyCycle = 0.0;

        // Applying configs
        talonFX.getConfigurator().apply(configs);
        talonFX.setPosition(0); //reset position
    }

    public Command runTorqueCurrent(double rpm) {
        return runOnce(() -> talonFX.setControl(dutyRequest.withVelocity(rpm)));
    }

    public Command runBangBangDuty(double rpm) {
        return runOnce(() -> talonFX.setControl(torqueCurrentRequest.withVelocity(rpm)));
    }
    public Command stop() {
        return runOnce(() -> talonFX.setControl(voltageRequest.withVelocity(0)));
    }

    // 50Hz NetworkTable variables
    // Creates a new field that contains all output variables
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable shooterTable = inst.getTable("Shooter");
    // Velocity
    private final DoublePublisher currentRpm = shooterTable.getDoubleTopic("Current ang vel (rpm)").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher targetRpm = shooterTable.getDoubleTopic("Target ang vel (rpm)").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher rpmDiff = shooterTable.getDoubleTopic("Ang vel diff (rpm)").publish(PubSubOption.periodic(0.02));
    
    // Accel
    private final DoublePublisher currentAccel = shooterTable.getDoubleTopic("Current ang accel (per min)").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher targetAccel = shooterTable.getDoubleTopic("Target ang accel (per min)").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher accelDiff = shooterTable.getDoubleTopic("Ang accel diff (per min)").publish(PubSubOption.periodic(0.02));

    @Override
    public void periodic() {
        double targetrpm = talonFX.getClosedLoopReference().getValueAsDouble();
        double currentrpm = talonFX.getVelocity().getValueAsDouble();

        // Field variable outputs
        // Velocity
        if (talonFX.getDeviceID() == 15) {
        currentRpm.set(currentrpm*60);
        targetRpm.set(targetrpm*60);
        rpmDiff.set((currentrpm - targetrpm)*60);
        // Acceleration
        currentAccel.set(talonFX.getAcceleration(true).getValueAsDouble());
        targetAccel.set(talonFX.getClosedLoopReferenceSlope(true).getValueAsDouble());
        accelDiff.set((talonFX.getAcceleration(true).getValueAsDouble() - talonFX.getClosedLoopReferenceSlope(true).getValueAsDouble())*60);
        }
    }
}

// CONSTANTS
/*
 * // Gains
        if (ID == 54) {
            slot0.kS = 0.11821;
            slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
            slot0.kV = 0.115;
            slot0.kA = 0.012983;
            slot0.kP = 0.14362; 
        } else if (ID == 52) {
            slot0.kS = 0.14611;
            slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
            slot0.kV = 0.116;
            slot0.kA = 0.013726;
            slot0.kP = 0.076057; 
        } else {
            slot0.kS = 0.12779;
            slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
            slot0.kV = 0.1145;
            slot0.kA = 0.012318;
            slot0.kP = 0.060303; 
        }
 */