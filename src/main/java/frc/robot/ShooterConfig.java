package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ShooterConfig extends SubsystemBase{

    private TalonFX talonFX; //ID?

    private TalonFXConfiguration talonFXconfigs;

    private MotionMagicVelocityVoltage request;

    private VoltageOut voltReq; // Only for SysID

    public ShooterConfig(int ID) {
        // Talon controls
        talonFX = new TalonFX(ID);

        talonFXconfigs = new TalonFXConfiguration();

        request = new MotionMagicVelocityVoltage(0);

        talonFX.setControl(request.withUpdateFreqHz(50));

        // INVERSION CHECK!!!!!!!
        talonFXconfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Limits and modes 
        talonFXconfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXconfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXconfigs.CurrentLimits.SupplyCurrentLimit = 40;

        // Gains
        talonFXconfigs.Slot0.kS = 0;
        talonFXconfigs.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        talonFXconfigs.Slot0.kV = 0;
        talonFXconfigs.Slot0.kA = 0;
        talonFXconfigs.Slot0.kP = 0; 
        talonFXconfigs.Slot0.kD = 0;

        // Motion Magic
        talonFXconfigs.MotionMagic.MotionMagicAcceleration = 800; // rot/sec^2
        talonFXconfigs.MotionMagic.MotionMagicJerk = 4000; // rot/sec^3

        // Applying configs
        talonFX.getConfigurator().apply(talonFXconfigs);

        talonFX.setPosition(0); //reset position
    }

    /** units of rot/s */
    public void setAngularVel(double vel) {
        talonFX.setControl(request.withVelocity(vel)); //rot/sec
        //r_leader.set(vel);
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
    private final DoublePublisher currentVel = shooterTable.getDoubleTopic("Current ang vel (rpm)").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher targetVel = shooterTable.getDoubleTopic("Target ang vel (rpm)").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher velDiff = shooterTable.getDoubleTopic("Ang vel diff (rpm)").publish(PubSubOption.periodic(0.02));
    
    // Accel
    private final DoublePublisher currentAccel = shooterTable.getDoubleTopic("Current ang accel (per min)").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher targetAccel = shooterTable.getDoubleTopic("Target ang accel (per min)").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher accelDiff = shooterTable.getDoubleTopic("Ang accel diff (per min)").publish(PubSubOption.periodic(0.02));

    @Override
    public void periodic() {
        // Field variable outputs
        // Velocity
        currentVel.set(talonFX.getVelocity(true).getValueAsDouble()*60);
        targetVel.set(talonFX.getClosedLoopReference(true).getValueAsDouble()*60);
        velDiff.set((talonFX.getVelocity(true).getValueAsDouble() - talonFX.getClosedLoopReference(true).getValueAsDouble())*60);
        // Acceleration
        currentAccel.set(talonFX.getAcceleration(true).getValueAsDouble());
        targetAccel.set(talonFX.getClosedLoopReferenceSlope(true).getValueAsDouble());
        accelDiff.set((talonFX.getAcceleration(true).getValueAsDouble() - talonFX.getClosedLoopReferenceSlope(true).getValueAsDouble())*60);
    }
}