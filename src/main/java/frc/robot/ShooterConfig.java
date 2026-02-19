package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ShooterConfig extends SubsystemBase{

    private TalonFX talonFX; //ID?
    private TalonFXConfiguration talonFXconfigs;
    private VelocityVoltage request;
    private VoltageOut voltReq; // Only for SysID

    private DutyCycleOut duty;
    private BangBangController bangBangController;
    private boolean reachedSetpoint;

    public ShooterConfig(int ID) {
        //54 is inverted!!!!!!!!!!
        // Talon controls
        talonFX = new TalonFX(ID);
        talonFXconfigs = new TalonFXConfiguration();
        request = new VelocityVoltage(0);
        talonFX.setControl(request.withUpdateFreqHz(50));
        voltReq = new VoltageOut(0.0);

        // control types
        duty = new DutyCycleOut(0);
        bangBangController = new BangBangController();
        
        // INVERSION CHECK!!!!!!!
        if (ID == 54)
            talonFXconfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        else
            talonFXconfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Limits and modes 
        talonFXconfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        talonFXconfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXconfigs.CurrentLimits.SupplyCurrentLimit = 40;

        Slot0Configs slot0 = talonFXconfigs.Slot0;
        // Gains
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

        // Applying configs
        talonFX.getConfigurator().apply(talonFXconfigs);

        talonFX.setPosition(0); //reset position

        reachedSetpoint = false;
    }

    /** units of rot/s */
    public void setAngularVel(double vel) {
        talonFX.setControl(request.withVelocity(vel)); //rot/sec
        // setpoint changes
        reachedSetpoint = false;
    }
    
    public boolean rpmWithin(double error) {
        double target = talonFX.getClosedLoopReference().getValueAsDouble();
        double current = talonFX.getVelocity().getValueAsDouble();

        if (Math.abs(target) < 1e-6) return false; // avoid divide by zero

        double relativeError = Math.abs(current - target) / Math.abs(target);
        if (relativeError < error) reachedSetpoint = true;

        return relativeError < error && reachedSetpoint;
    }

    // SysID
    private SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, // Default ramp rate (1V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
            null, // Default timeout (10s)

            (state) -> SignalLogger.writeString("State", state.toString())
        ), 
        new SysIdRoutine.Mechanism(
            (volts) -> talonFX.setControl(voltReq.withOutput(volts.in(Volts))),
            null, // Left null when using a signal logger
            this
        )
    );

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
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

        // within 0.67% rpm of error
        if (rpmWithin(0.0067)) {
            double output = bangBangController.calculate(currentrpm, targetrpm);
            talonFX.setControl(duty.withOutput(output));
        } else talonFX.setControl(request.withVelocity(targetrpm));

        // Field variable outputs
        // Velocity
        if (talonFX.getDeviceID() == 55) {
        currentRpm.set(targetrpm*60);
        targetRpm.set(targetrpm*60);
        rpmDiff.set((currentrpm - targetrpm)*60);
        // Acceleration
        currentAccel.set(talonFX.getAcceleration(true).getValueAsDouble());
        targetAccel.set(talonFX.getClosedLoopReferenceSlope(true).getValueAsDouble());
        accelDiff.set((talonFX.getAcceleration(true).getValueAsDouble() - talonFX.getClosedLoopReferenceSlope(true).getValueAsDouble())*60);
        }
    }
}