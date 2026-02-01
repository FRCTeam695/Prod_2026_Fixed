package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VoltageOut;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;


public class Intake extends SubsystemBase {
    private TalonFX pivot;
    private TalonFX roller;

    private MotionMagicVoltage mm_pivot; //maybe make better name?

    private static final double kPivotReduction = 0; //make into constant, amount of rotations for arm to loop once

    private NetworkTableInstance inst;
    private NetworkTable table;
    private NetworkTableEntry pivotValue;
    private NetworkTableEntry rollerValue;

    //private VoltageOut pivotVoltageRequest;
    //private VoltageOut rollerVoltageRequest;

    public Intake() {
        pivot = new TalonFX(0); //not correct id
        roller = new TalonFX(1); //not correct id

        configurePivot();
        configureRoller();

        mm_pivot = new MotionMagicVoltage(0).withSlot(0);

        //pivotVoltageRequest = new VoltageOut(0); //output is zero
        //rollerVoltageRequest = new VoltageOut(0);

        //networktables
        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("Values");
        pivotValue = table.getEntry("Pivot Position");
        rollerValue = table.getEntry("Roller Speed");
    }
    
    public void configurePivot() {
        TalonFXConfiguration config1 = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(20.0)
            )
            .withFeedback(
                new FeedbackConfigs()
                .withSensorToMechanismRatio(kPivotReduction)
            )
            .withMotionMagic(
                new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(0) //NEEDS TO BE CHANGED
                .withMotionMagicAcceleration(0) //NEEDS TO BE CHANGED
            )
            .withSlot0(
                new Slot0Configs()
                .withKP(0) //NEED TO CHANGE
                .withKI(0) //NEED TO CHANGE (add kd and kv if needed)
            );
        pivot.getConfigurator().apply(config1);
    }

    public void configureRoller() {
        TalonFXConfiguration config2 = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(20.0)
            )
            .withFeedback(
                new FeedbackConfigs()
                .withSensorToMechanismRatio(kPivotReduction)
            );
        roller.getConfigurator().apply(config2);
    }

    public Command setPivot(double pivotVal) {
        return runOnce(
            () -> {
                pivot.setControl(
                    mm_pivot.withPosition(pivotVal) // make into VARIABLE use Degrees.of(0)
                );
                
                pivotValue.setDouble(
                    pivot.getPosition().getValue().in(Degrees)
                );
            }
        );
    }

    public Command setRollerPercent(double percent) { //no motion magic
        return runOnce(
            () -> {
                roller.set(percent);

                double rollerOutput = roller.getVelocity().getValue().in(Units.RotationsPerSecond);
                rollerValue.setDouble(rollerOutput);
            }
        );
    }

    public Command intake() { //fix this
        return startEnd(
            () -> {
                setPivot(intakeConstants.INTAKE_PIVOT);
                setRollerPercent(intakeConstants.INTAKE_ROLLER);
            },
            () -> setRollerPercent(0)
        );
    }

}
