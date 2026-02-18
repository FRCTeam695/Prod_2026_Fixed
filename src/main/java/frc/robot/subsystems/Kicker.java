package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Kicker extends SubsystemBase {

    // 1 motor Rotation       12 teeth         1 rot        1.625 inches (diameter)   pi (circumference)       1 meter
    // ----------------  *  ------------  *  ----------  *  ----------------------- * ------------------- * -------------
    //        1                1 rot          36 teeth               1 rot                1 diameter         39.37 inches
    public final double surfaceMetersPerMotorRotation = 12.0 / 36.0 * 1.625 * Math.PI / 39.37;
    public final double maxSpeedRPS = 100.;

    private final TalonFX motor;

    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    private final double kMaxRPM = 6000.0;

    final VelocityVoltage m_velocity;
    final VoltageOut m_voltage;

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable feederTable = inst.getTable("Kicker");

    private final DoublePublisher dutyCyclePub = feederTable.getDoubleTopic("Kicker Duty Cycle").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher setPointPub = feederTable.getDoubleTopic("Kicker Set Point (Meters Per Sec)").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher velocityPub = feederTable.getDoubleTopic("Kicker Current Velocity (Meters Per Sec)").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher outputVoltagePub = feederTable.getDoubleTopic("Kicker Output Voltage (Volts)").publish(PubSubOption.periodic(0.02));

    public Kicker() {
        motor = new TalonFX(51);

        final TalonFXConfiguration config = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(120))
                        .withSupplyCurrentLimit(40)
                        .withSupplyCurrentLimitEnable(true)
                        .withStatorCurrentLimitEnable(true));

        motor.getConfigurator().apply(config);
        SmartDashboard.putData(this);
    

        // class member variable
        m_velocity = new VelocityVoltage(0);
        m_voltage = new VoltageOut(0);

        // robot init, set slot 0 gains
        var slot0Configs = new Slot0Configs();
        slot0Configs.kV = 9.8 / 83.5;
        slot0Configs.kP = 0.3;
        // slot0Configs.kI = 0.48;
        // slot0Configs.kD = 0.01;
        slot0Configs.kS = 0.2; //V
        motor.getConfigurator().apply(slot0Configs, 0.050);
    }

    public void runBangBang(double targetRPM) {
        double currentRPM = motor.getVelocity().getValue().in(RPM);

        if (targetRPM > 50 && currentRPM < targetRPM) {
            motor.setControl(dutyCycleRequest.withOutput(1.0));
        } else {
            stop();
        }
    }

    public Command joystickBangBangCommand(DoubleSupplier speedSupplier) {
        return run(() -> {
            double target = Math.abs(speedSupplier.getAsDouble()) * kMaxRPM;
            runBangBang(target);
        }).withName("JoystickBangBang");
    }

    public double motorRotationsFromSurfaceMeters(double surfaceMeters) {
        return surfaceMeters / surfaceMetersPerMotorRotation;
    }

    public double surfaceMetersFromMotorRotations(double motorRotations) {
        return motorRotations * surfaceMetersPerMotorRotation;
    }

    public Command setVelocityMPS(DoubleSupplier velocityMPS) {
        return setVelocityRPS(() -> motorRotationsFromSurfaceMeters(velocityMPS.getAsDouble()));
    }

    private Command setVelocityRPS(DoubleSupplier velocityRPS) {
        return run(() -> {
            motor.setControl(m_velocity.withVelocity(velocityRPS.getAsDouble()));
        });
    }

    public Command setDutyCycle(DoubleSupplier dutyCycle) {
        return run(() -> {
            motor.set(dutyCycle.getAsDouble());
        });
    }

    public Command setVoltage(DoubleSupplier voltage) {
        return run(() -> {
            motor.setControl(m_voltage.withOutput(voltage.getAsDouble()));
        });
    }

    public Command stop() {
        return runOnce(() -> {
            motor.setControl(dutyCycleRequest.withOutput(0));
        });
    }

    @Override
    public void periodic() {
        dutyCyclePub.set(motor.getDutyCycle().getValueAsDouble());
        setPointPub.set(surfaceMetersFromMotorRotations(motor.getClosedLoopReference().getValueAsDouble()));
        velocityPub.set(surfaceMetersFromMotorRotations(motor.getVelocity().getValueAsDouble()));
        outputVoltagePub.set(motor.getMotorVoltage().getValueAsDouble());
    }
}