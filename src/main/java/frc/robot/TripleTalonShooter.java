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

public class TripleTalonShooter extends SubsystemBase{

    // Talons (r, m, l)
    private TalonFX r_talon; //ID?
    private TalonFX m_talon; //ID?
    private TalonFX l_talon; //ID?

    private MotionMagicVelocityVoltage r_requests;
    private MotionMagicVelocityVoltage m_requests;
    private MotionMagicVelocityVoltage l_requests;

    private TalonFXConfiguration r_configs;
    private TalonFXConfiguration m_configs;
    private TalonFXConfiguration l_configs;

    private VoltageOut r_voltReq; // Only for SysID
    private VoltageOut m_voltReq;
    private VoltageOut l_voltReq;

    // Constructor
    public TripleTalonShooter() {
        // Talon controls
        r_talon = new TalonFX(1); //ID?
        m_talon = new TalonFX(2); //ID?
        l_talon = new TalonFX(3); //ID?

        r_requests = new MotionMagicVelocityVoltage(0);
        m_requests = new MotionMagicVelocityVoltage(0);
        l_requests = new MotionMagicVelocityVoltage(0);

        r_configs = new TalonFXConfiguration();
        m_configs = new TalonFXConfiguration();
        l_configs = new TalonFXConfiguration();

        r_talon.setControl(r_requests.withUpdateFreqHz(50));
        m_talon.setControl(m_requests.withUpdateFreqHz(50));
        l_talon.setControl(l_requests.withUpdateFreqHz(50));

        // INVERSION!!!!!!!
        r_configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        l_configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Limits and modes 
        r_configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_configs.MotorOutput.NeutralMode = NeutralModeValue.Brake; 
        l_configs.MotorOutput.NeutralMode = NeutralModeValue.Brake; 

        r_configs.CurrentLimits.SupplyCurrentLimitEnable = true;
        r_configs.CurrentLimits.SupplyCurrentLimit = 40;
        m_configs.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_configs.CurrentLimits.SupplyCurrentLimit = 40;
        l_configs.CurrentLimits.SupplyCurrentLimitEnable = true;
        l_configs.CurrentLimits.SupplyCurrentLimit = 40;

        // Tuning
        r_configs.Slot0 = r_configs.Slot0;
        m_configs.Slot0 = m_configs.Slot0;
        l_configs.Slot0 = l_configs.Slot0;

        r_configs.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        m_configs.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        l_configs.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        r_configs.Slot0.kS = 0;
        r_configs.Slot0.kV = 0;
        r_configs.Slot0.kA = 0;
        r_configs.Slot0.kP = 0; 
        r_configs.Slot0.kD = 0;

        m_configs.Slot0.kS = 0;
        m_configs.Slot0.kV = 0;
        m_configs.Slot0.kA = 0;
        m_configs.Slot0.kP = 0; 
        m_configs.Slot0.kD = 0;

        l_configs.Slot0.kS = 0;
        l_configs.Slot0.kV = 0;
        l_configs.Slot0.kA = 0;
        l_configs.Slot0.kP = 0; 
        l_configs.Slot0.kD = 0;

        // Motion Magic
        r_configs.MotionMagic.MotionMagicAcceleration = 800; // rot/sec^2
        r_configs.MotionMagic.MotionMagicJerk = 4000; // rot/sec^3

        m_configs.MotionMagic.MotionMagicAcceleration = 800;
        m_configs.MotionMagic.MotionMagicJerk = 4000;
        
        l_configs.MotionMagic.MotionMagicAcceleration = 800;
        l_configs.MotionMagic.MotionMagicJerk = 4000;

        // Applying configs
        r_talon.getConfigurator().apply(r_configs);
        m_talon.getConfigurator().apply(m_configs);
        l_talon.getConfigurator().apply(l_configs);

        r_talon.setPosition(0); //reset position
        m_talon.setPosition(0); //reset position
        l_talon.setPosition(0); //reset position

    }

    public void setVel(double vel, String motor) {
        switch (motor) {
            case "r":
                r_talon.setControl(r_requests.withVelocity(vel)); //rot/sec
            case "m":
                m_talon.setControl(m_requests.withVelocity(vel));
            default:
                l_talon.setControl(l_requests.withVelocity(vel));
        }
        //r_leader.set(vel);
    }

    // SysID
    private SysIdRoutine r_sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, // Default ramp rate (1V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
            null, // Default timeout (10s)

            (state) -> SignalLogger.writeString("State", state.toString())
        ), 
        new SysIdRoutine.Mechanism(
            (volts) -> r_talon.setControl(r_voltReq.withOutput(volts.in(Volts))),
            null, // Left null when using a signal logger
            this
        )
    );

        private SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, // Default ramp rate (1V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
            null, // Default timeout (10s)

            (state) -> SignalLogger.writeString("State", state.toString())
        ), 
        new SysIdRoutine.Mechanism(
            (volts) -> m_talon.setControl(r_voltReq.withOutput(volts.in(Volts))),
            null, // Left null when using a signal logger
            this
        )
    );

        private SysIdRoutine l_sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, // Default ramp rate (1V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
            null, // Default timeout (10s)

            (state) -> SignalLogger.writeString("State", state.toString())
        ), 
        new SysIdRoutine.Mechanism(
            (volts) -> l_talon.setControl(r_voltReq.withOutput(volts.in(Volts))),
            null, // Left null when using a signal logger
            this
        )
    );

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction, String system) {
        switch (system) {
            case "r":
                return r_sysIdRoutine.quasistatic(direction);
            case "m":
                return m_sysIdRoutine.quasistatic(direction);
            default:
                return l_sysIdRoutine.quasistatic(direction);
        }
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction, String system) {
        switch (system) {
            case "r":
                return r_sysIdRoutine.dynamic(direction);
            case "m":
                return m_sysIdRoutine.dynamic(direction);
            default:
                return l_sysIdRoutine.dynamic(direction);
        }
    }

    // 50Hz NetworkTable variables
    // Creates a new field that contains all output variables
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable shooterTable = inst.getTable("Shooter");
    // Velocity
    private final DoublePublisher r_vel = shooterTable.getDoubleTopic("r_vel").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher r_velTarget = shooterTable.getDoubleTopic("r_velTarget").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher r_velDiff = shooterTable.getDoubleTopic("r_velDiff").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher m_vel = shooterTable.getDoubleTopic("m_vel").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher m_velTarget = shooterTable.getDoubleTopic("m_velTarget").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher m_velDiff = shooterTable.getDoubleTopic("m_velDiff").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher l_vel = shooterTable.getDoubleTopic("l_vel").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher l_velTarget = shooterTable.getDoubleTopic("l_velTarget").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher l_velDiff = shooterTable.getDoubleTopic("l_velDiff").publish(PubSubOption.periodic(0.02));
    // Accel
    private final DoublePublisher r_accel = shooterTable.getDoubleTopic("r_accel").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher r_accelTarget = shooterTable.getDoubleTopic("r_accelTarget").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher r_accelDiff = shooterTable.getDoubleTopic("r_accelDiff").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher m_accel = shooterTable.getDoubleTopic("m_accel").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher m_accelTarget = shooterTable.getDoubleTopic("m_accelTarget").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher m_accelDiff = shooterTable.getDoubleTopic("m_accelDiff").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher l_accel = shooterTable.getDoubleTopic("l_accel").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher l_accelTarget = shooterTable.getDoubleTopic("l_accelTarget").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher l_accelDiff = shooterTable.getDoubleTopic("l_accelDiff").publish(PubSubOption.periodic(0.02));

    @Override
    public void periodic() {
        // Field variable outputs
        // Velocity
        r_vel.set(r_talon.getVelocity(true).getValueAsDouble()*60);
        r_velTarget.set(r_talon.getClosedLoopReference(true).getValueAsDouble()*60);
        r_velDiff.set((r_talon.getVelocity(true).getValueAsDouble() - r_talon.getClosedLoopReference(true).getValueAsDouble())*60);
        m_vel.set(m_talon.getVelocity(true).getValueAsDouble()*60);
        m_velTarget.set(m_talon.getClosedLoopReference(true).getValueAsDouble()*60);
        m_velDiff.set((m_talon.getVelocity(true).getValueAsDouble() - m_talon.getClosedLoopReference(true).getValueAsDouble())*60);
        l_vel.set(l_talon.getVelocity(true).getValueAsDouble()*60);
        l_velTarget.set(l_talon.getClosedLoopReference(true).getValueAsDouble()*60);
        l_velDiff.set((l_talon.getVelocity(true).getValueAsDouble() - l_talon.getClosedLoopReference(true).getValueAsDouble())*60);
        // Acceleration
        r_accel.set(r_talon.getAcceleration(true).getValueAsDouble());
        r_accelTarget.set(r_talon.getClosedLoopReferenceSlope(true).getValueAsDouble());
        r_accelDiff.set((r_talon.getAcceleration(true).getValueAsDouble() - r_talon.getClosedLoopReferenceSlope(true).getValueAsDouble())*60);
        m_accel.set(m_talon.getAcceleration(true).getValueAsDouble());
        m_accelTarget.set(m_talon.getClosedLoopReferenceSlope(true).getValueAsDouble());
        m_accelDiff.set((m_talon.getAcceleration(true).getValueAsDouble() - m_talon.getClosedLoopReferenceSlope(true).getValueAsDouble())*60);
        l_accel.set(l_talon.getAcceleration(true).getValueAsDouble());
        l_accelTarget.set(l_talon.getClosedLoopReferenceSlope(true).getValueAsDouble());
        l_accelDiff.set((l_talon.getAcceleration(true).getValueAsDouble() - l_talon.getClosedLoopReferenceSlope(true).getValueAsDouble())*60);
    }
}