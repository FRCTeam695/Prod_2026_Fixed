package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
//import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

//import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.intakeConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import static edu.wpi.first.units.Units.Degrees;

import static edu.wpi.first.units.Units.Amps;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;


public class Intake_Roller extends SubsystemBase {
    private TalonFX roller;

    private NetworkTableInstance inst;
    private NetworkTable table;
    private NetworkTableEntry rollerSetValue;
    private NetworkTableEntry rollerActValue;

    private final VelocityVoltage velocity = new VelocityVoltage(0);

    // private boolean isHomed = false;

    public Intake_Roller() {
        roller = new TalonFX(14); //not correct id
        configureRoller();

        //networktables
        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("Values");
        rollerSetValue = table.getEntry("Roller Setpoint");
        rollerActValue = table.getEntry("Roller Actual");
    }

    public void configureRoller() {
        TalonFXConfiguration config2 = new TalonFXConfiguration()
            .withMotorOutput(
               new MotorOutputConfigs()
               .withInverted(InvertedValue.Clockwise_Positive)
               .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(120))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(70))
                .withSupplyCurrentLimitEnable(true)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(0)
                    .withKI(0)
                    .withKD(0)
                    .withKV(0)
            );
        roller.getConfigurator().apply(config2);
    }

    public Command setRollerVelocity(double targetVelocity) {
        return run(() -> roller.setControl(velocity.withVelocity(targetVelocity)));
    }

    @Override
    public void periodic(){
        double r_actual = roller.getVelocity().getValueAsDouble();
        double r_setpoint = velocity.Velocity;
        
        rollerActValue.setDouble(r_actual);
        rollerSetValue.setDouble(r_setpoint);
        
        SmartDashboard.putNumber("Actual Velocity", r_actual);
        SmartDashboard.putNumber("Set Velocity", r_setpoint);
    }
}
