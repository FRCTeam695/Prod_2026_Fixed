package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.Amps;

import java.util.function.DoubleSupplier;

public class IntakePivot extends SubsystemBase {
    private TalonFX pivot;

    private MotionMagicVoltage motionMagicSetter;
    private DutyCycleOut dutyCycleSetter;

    private static final double kPivotReduction = 46.4; //make into constant, amount of rotations for arm to loop once

    private NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private NetworkTable table = inst.getTable("Intake Pivot");

    private final DoublePublisher currentPositionDegreesPub = table.getDoubleTopic("Current Position Degrees").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher setpointPositionDegreesPub = table.getDoubleTopic("Setpoint Position Degrees").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher dutyCyclePub = table.getDoubleTopic("Duty Cycle").publish(PubSubOption.periodic(0.02));
    private final BooleanPublisher atSetpointPub = table.getBooleanTopic("Intake Pivot at Setpoint").publish(PubSubOption.periodic(0.02));;

    public final double pivotRetractedPositionDegrees = 110.16;
    public final double pivotExtendedPositionDegrees = 1.005;//5.8886; //3.25 = 1shim, 1.005=2shim, 5.8886
    public final double pivotAgitatePositionDegrees = 50;

    public final Trigger withinTolerance;
    public final Trigger statorOverThreshold;
    public final Trigger velocityAtZero;

    private final double pivotTolerance = 5;
    private final double typicalCurrentLimit = 50;
    private final double agitateDegreeError = 10;
    private final double statorAmpLimit = 20;


        public IntakePivot() {
    
            // as intake moves from retracted to extended, rotations will go closer to 0 (horizontal=0)
    
            pivot = new TalonFX(57);
            configurePivot();
            motionMagicSetter = new MotionMagicVoltage(0).withSlot(0);
            dutyCycleSetter = new DutyCycleOut(0);
    
            pivot.setPosition(Units.degreesToRotations(pivotRetractedPositionDegrees));
            motionMagicSetter.withPosition(pivotRetractedPositionDegrees);

            withinTolerance = new Trigger(
                ()-> Math.abs(Units.rotationsToDegrees(motionMagicSetter.Position - pivot.getPosition().getValueAsDouble())) < pivotTolerance
            );

            statorOverThreshold = new Trigger(
                ()-> pivot.getStatorCurrent().getValueAsDouble() > statorAmpLimit
            );

            velocityAtZero = new Trigger(
                ()-> pivot.getVelocity().getValueAsDouble() < 0.02
            );

        }
        
        public void configurePivot() {
            TalonFXConfiguration config = new TalonFXConfiguration()
                .withMotorOutput(
                    new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake)
                )
                .withCurrentLimits( 
                    new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(typicalCurrentLimit))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(30)) // fix magic number
                    .withSupplyCurrentLimitEnable(true)
                )
                .withFeedback(
                    new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(kPivotReduction)
                )
                .withMotionMagic(
                    new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(Units.degreesToRotations(500)) 
                    .withMotionMagicAcceleration(Units.degreesToRotations(600))
                )
                .withSlot0(
                    new Slot0Configs()
                    .withKP(10) //NEED TO CHANGE
                    .withKI(0) //NEED TO CHANGE (add kd and kv if needed)
                    .withKV(5.51)
                    .withKG(0.3)
                    .withKS(0.06)
                    .withKA(0.11)
                    .withGravityType(GravityTypeValue.Arm_Cosine)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
                ).withSoftwareLimitSwitch(
                    new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitThreshold(Units.degreesToRotations(pivotRetractedPositionDegrees))
                    .withReverseSoftLimitThreshold(Units.degreesToRotations(pivotExtendedPositionDegrees))
                    .withForwardSoftLimitEnable(true)
                    .withReverseSoftLimitEnable(true)
                );
            pivot.getConfigurator().apply(config);
        }

        /*
         * Moves the intake up until it can't move anymore.
         * Moves the intake down a specified degree amount to allow fuel to wiggle.
         * Repeats.
         */
        public Command agitateWithDutyCycleAndDegreeError(){
            return (
                run(()->{
                    pivot.set(0.1);
                })
                .until(
                    statorOverThreshold.or(velocityAtZero)
                )
                .andThen(new ConditionalCommand(
                    // if intake has enough room to move backwards, send it back by agitateDegreeError
                    goToPositionDegreesWithCondition(pivot.getPosition().getValueAsDouble() - agitateDegreeError, withinTolerance),

                    // if intake doesn't have enough room to move backwards by agitateDegreeError, return to extended position
                    goToPositionDegreesWithCondition(pivotExtendedPositionDegrees, withinTolerance),
                    
                    () -> Units.rotationsToDegrees(pivot.getPosition().getValueAsDouble()) > agitateDegreeError
                )))
                .repeatedly();        
        }

        public Command slowRaise(){
            return run(()-> {
                pivot.setControl(dutyCycleSetter.withOutput(0.05));
            }).until(
                ()-> Units.rotationsToDegrees(pivot.getPosition().getValueAsDouble()) > 23
            )
            .andThen(
                setPositionDegrees(()-> 27)
            );
        }

        public Command goToPositionDegreesWithCondition(double degrees, Trigger condition){
            return runOnce(()-> {
                pivot.setControl(motionMagicSetter.withPosition(Units.degreesToRotations(degrees)));
            })
            .andThen(
                run(()-> {
                    pivot.setControl(motionMagicSetter.withPosition(Units.degreesToRotations(degrees)));
                }).until(condition)
            );
        }


        public Command setPositionDegrees(DoubleSupplier angleDegrees) {
            return run(
                () -> {
                    pivot.setControl(motionMagicSetter.withPosition(Units.degreesToRotations(angleDegrees.getAsDouble())));
                }
            );
        }

        public Command setDutyCycle(DoubleSupplier velocity) {
            return run(
                () -> {
                    pivot.set(velocity.getAsDouble());
                }
            );
        }

    public Command homePivotToRetracted(){
        return
        (run ( () -> {

            pivot.setControl(dutyCycleSetter.withOutput(0.05).withIgnoreSoftwareLimits(true));
        }))
        .until(
            () -> 
            pivot.getStatorCurrent().getValueAsDouble() > 15
        )
        .andThen(
            runOnce(
                () -> pivot.setPosition(Units.degreesToRotations(pivotRetractedPositionDegrees))
        ))
        .andThen(
            run ( () -> pivot.setControl(dutyCycleSetter.withOutput(0)))
        ); // make the StatorCurrentLimit (15) into a constant
    }

    public Command homePivotToExtended(){
        return
        (run ( () -> {

            pivot.setControl(dutyCycleSetter.withOutput(-0.05).withIgnoreSoftwareLimits(true));
        }))
        .until(
            () -> 
            pivot.getStatorCurrent().getValueAsDouble() > 15
        )
        .andThen(
            runOnce(
                () -> pivot.setPosition(Units.degreesToRotations(pivotExtendedPositionDegrees))
        ))
        .andThen(
            run ( () -> pivot.setControl(dutyCycleSetter.withOutput(0)))
        ); // make the StatorCurrentLimit (15) into a constant
    }

    public Command setPivotStartingPositionToExtended(){
        return runOnce( () -> {
            pivot.setPosition(Units.degreesToRotations(pivotExtendedPositionDegrees));
         });
    }
   
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Voltage", pivot.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Current", pivot.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Motor Stall Current", pivot.getMotorStallCurrent().getValueAsDouble());

        currentPositionDegreesPub.set(Units.rotationsToDegrees(pivot.getPosition().getValueAsDouble()));
        setpointPositionDegreesPub.set(Units.rotationsToDegrees(pivot.getClosedLoopReference().getValueAsDouble()));
        dutyCyclePub.set(pivot.getDutyCycle().getValueAsDouble());
        atSetpointPub.set(withinTolerance.getAsBoolean());

        SmartDashboard.putBoolean("Stator Current Trigger", statorOverThreshold.getAsBoolean());
    }
}