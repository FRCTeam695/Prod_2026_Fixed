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
import edu.wpi.first.math.util.*;
import edu.wpi.first.wpilibj2.command.Command;
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
    private final DoublePublisher statorCurrentPub = table.getDoubleTopic("Stator Current").publish(PubSubOption.periodic(0.02));
    private final BooleanPublisher atSetpointPub = table.getBooleanTopic("Intake Pivot at Setpoint").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher voltagePub = table.getDoubleTopic("Voltage").publish();
    private final DoublePublisher stallCurrentPub = table.getDoubleTopic("Stall Current").publish();
    private final BooleanPublisher statorOverThresholdPub = table.getBooleanTopic("Stator Over 20 A").publish();

    public final double pivotRetractedPositionDegrees = 134.9;
    public final double pivotExtendedPositionDegrees = 1.005;//5.8886; //3.25 = 1shim, 1.005=2shim, 5.8886
    public final double pivotAgitatePositionDegrees = 50;

    public final Trigger withinTolerance;
    public final Trigger statorOverThreshold;
    public final Trigger velocityAtZero;

    private final double pivotTolerance = 10;
    private final double statorAmpLimit = 20;

    private final double pivotRaiseLimitDeg = 35;


        public IntakePivot() {
    
            // as intake moves from retracted to extended, rotations will go closer to 0 (horizontal=0)
    
            pivot = new TalonFX(57);
            configurePivot();
            motionMagicSetter = new MotionMagicVoltage(0).withSlot(0);
            dutyCycleSetter = new DutyCycleOut(0);
            pivot.getStatorCurrent().setUpdateFrequency(50);
    
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
                    .withStatorCurrentLimit(Amps.of(statorAmpLimit))
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

        public Command slowRaise(){
            return 
            (
                run(()->{
                pivot.setControl(dutyCycleSetter.withOutput(0.1));
            })
            .until(
                () -> (Units.rotationsToDegrees(pivot.getPosition().getValueAsDouble()) > pivotRaiseLimitDeg)
            )
            ).andThen(
                holdPosition()
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

        public Command holdPosition(){
            return run(
                () -> {
                    pivot.setControl(motionMagicSetter.withPosition(Units.degreesToRotations(pivotRaiseLimitDeg)));
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
   
    @Override
    public void periodic(){
        voltagePub.set(pivot.getMotorVoltage().getValueAsDouble());
        stallCurrentPub.set(pivot.getMotorStallCurrent().getValueAsDouble());
        statorCurrentPub.set(pivot.getStatorCurrent().getValueAsDouble());
        statorOverThresholdPub.set(statorOverThreshold.getAsBoolean());
        dutyCyclePub.set(pivot.getDutyCycle().getValueAsDouble());

        currentPositionDegreesPub.set(Units.rotationsToDegrees(pivot.getPosition().getValueAsDouble()));
        setpointPositionDegreesPub.set(Units.rotationsToDegrees(pivot.getClosedLoopReference().getValueAsDouble()));
        atSetpointPub.set(withinTolerance.getAsBoolean());
    }
}