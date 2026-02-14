package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;


import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase{
    public enum Speed {
        FEED(5000);

        private final double rpm;

        private Speed(double rpm) {
            this.rpm = rpm;
        }

        public AngularVelocity angularVelocity() {
            return RPM.of(rpm);
        }
    }

    private final TalonFX motor;
    private final TalonFX motor2;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    BangBangController controller;
    double setpoint;
    SimpleMotorFeedforward feedforward;

    public Feeder() {

        setpoint = 50;
        feedforward = new SimpleMotorFeedforward(0.1, 0.1, 0,1);
        controller = new BangBangController();

        
        //12
        motor = new TalonFX(22);
        motor2 = new TalonFX(15);

        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(50))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(1)
                    .withKI(0)
                    .withKD(0)
                    .withKV(12.0 / RPM.of(6000).in(RotationsPerSecond)) // 12 volts when requesting max RPS
            );
        
        motor.getConfigurator().apply(config);
        motor2.getConfigurator().apply(config);
        SmartDashboard.putData(this);
    }

    public void set(Speed speed) {
        motor.setControl(
            velocityRequest
                .withVelocity(speed.angularVelocity())
        );
    }

    public void setPercentOutput(double percentOutput) {
        motor.setControl(
            voltageRequest
                .withOutput(Volts.of(percentOutput * 12.0))
        );
    }
    
    public Command feedCommand() {
        return startEnd(() -> set(Speed.FEED), () -> setPercentOutput(0));
    }

    public Command moveMotor() {
        return runOnce(() -> {
            motor.set(1.0);
            motor2.set(1.0);
        });
    }

    public Command stopMotor() {
        return runOnce(() -> {
            motor.set(0.0);
            motor2.set(0.0);
        });
    }

    public Command bangbangTune() {
        return runOnce(() -> {
            motor.setVoltage(controller.calculate(motor.getVelocity().getValueAsDouble(), setpoint) * 12.0 + 0.9 * feedforward.calculate(setpoint));
            motor2.setVoltage(controller.calculate(motor.getVelocity().getValueAsDouble(), setpoint) * 12.0 + 0.9 * feedforward.calculate(setpoint));
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Motor1 Velocity", motor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Motor2 Velocity", motor2.getVelocity().getValueAsDouble());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("RPM", () -> motor.getVelocity().getValue().in(RPM), null);
        builder.addDoubleProperty("Stator Current", () -> motor.getStatorCurrent().getValue().in(Amps), null);
        builder.addDoubleProperty("Supply Current", () -> motor.getSupplyCurrent().getValue().in(Amps), null);
    } 
}
