package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
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
    private VoltageOut voltageSetter;
    private DutyCycleOut dutyCycleSetter;

    private static final double kPivotReduction = 46.4; //make into constant, amount of rotations for arm to loop once

    private NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private NetworkTable table = inst.getTable("Intake Pivot");

    private final DoublePublisher currentPositionDegreesPub = table.getDoubleTopic("Current Position Degrees").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher setpointPositionDegreesPub = table.getDoubleTopic("Setpoint Position Degrees").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher dutyCyclePub = table.getDoubleTopic("Duty Cycle").publish(PubSubOption.periodic(0.02));
    private final BooleanPublisher atSetpointPub = table.getBooleanTopic("Intake Pivot at Setpoint").publish(PubSubOption.periodic(0.02));;

    public final double pivotRetractedPositionDegrees = 110.16;
    public final double pivotExtendedPositionDegrees = 3.96;
    public final double pivotAgitatePositionDegrees = 50;

    public final Trigger atSetpoint;
    public final Trigger statorOverThreshold;
    public final Trigger velocityAtZero;

    private final double pivotTolerance = 5;
    private final double typicalCurrentLimit = 50.;
    private final double agitateDegreeError = 20.;
    private final double statorAmpLimit = 20.;


        public IntakePivot() {
    
            // as intake moves from retracted to extended, rotations will go closer to 0 (horizontal=0)
    
            pivot = new TalonFX(57);
            configurePivot();
            motionMagicSetter = new MotionMagicVoltage(0).withSlot(0);
            voltageSetter = new VoltageOut(0);
            dutyCycleSetter = new DutyCycleOut(0);
    
            pivot.setPosition(Units.degreesToRotations(pivotRetractedPositionDegrees));

            atSetpoint = new Trigger(
                ()-> Math.abs(Units.rotationsToDegrees(motionMagicSetter.Position - pivot.getPosition().getValueAsDouble())) < pivotTolerance
            );

            statorOverThreshold = new Trigger(
                ()-> pivot.getStatorCurrent().getValueAsDouble() > statorAmpLimit
            );

            velocityAtZero = new Trigger(
                ()-> Math.abs(pivot.getVelocity().getValueAsDouble()) < 0.02
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
        public Command agitateWithDutyCycleAndDegreeErrorFake(){
            return (
                run(()->{
                    pivot.set(0.1);
                })
                .until(
                    //statorOverThreshold
                    // .or(velocityAtZero)
                    statorOverThreshold.or(()-> (Units.rotationsToDegrees(pivot.getPosition().getValueAsDouble()) > 40))
                    
                )
                .andThen(
                        run(()-> {
                            pivot.set(-0.1);
                        })
                        .until(
                            ()-> Units.rotationsToDegrees(pivot.getPosition().getValueAsDouble()) < 15
                        )


                )
                )
                .repeatedly().handleInterrupt(()-> pivot.set(0));        
        }

         public Command agitateWithDutyCycleAndDegreeError(){
            return (
                run(()->{
                    pivot.set(0.1);
                })
                .until(
                    statorOverThreshold
                    .or((()-> Units.rotationsToDegrees(pivot.getPosition().getValueAsDouble()) > 45))
                )
                .andThen(new ConditionalCommand(
                    // if intake has enough room to move backwards, send it back by agitateDegreeError
                    goToAngleWithTrigger(Units.rotationsToDegrees(pivot.getPosition().getValueAsDouble()) - agitateDegreeError, atSetpoint),

                    // if intake doesn't have enough room to move backwards by agitateDegreeError, return to extended position
                    goToAngleWithTrigger(pivotExtendedPositionDegrees, atSetpoint),
                    
                    () -> Units.rotationsToDegrees(Math.abs(pivot.getPosition().getValueAsDouble()) - agitateDegreeError) > 0
                )))
                .repeatedly();        
        }

        public Command goToAngleWithTrigger(double angle, Trigger condition){
            return
            runOnce(()-> pivot.setControl(motionMagicSetter.withPosition(Units.degreesToRotations(angle))))
            .andThen(
                run(()-> {
                   pivot.setControl(motionMagicSetter.withPosition(Units.degreesToRotations(angle)));
                }).until(condition)
            );
        }



    public Command setVoltage(DoubleSupplier voltage){
        return run(
            () -> {
                pivot.setControl(voltageSetter.withOutput(voltage.getAsDouble()));
            }
        );
    }

    public Command agitateToConstantDegreeValue(){
            return
            ( run(
                    () -> {
                        pivot.setControl(motionMagicSetter.withPosition(Units.degreesToRotations(45)));
                    }
                )
            .until(
                    atSetpoint.or(statorOverThreshold)
            )
            .andThen( run (()->{
                pivot.setControl(motionMagicSetter.withPosition(Units.degreesToRotations(pivotExtendedPositionDegrees)));
        }).until(
                atSetpoint
            ))).repeatedly();
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

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Voltage", pivot.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Current", pivot.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Motor Stall Current", pivot.getMotorStallCurrent().getValueAsDouble());

        currentPositionDegreesPub.set(Units.rotationsToDegrees(pivot.getPosition().getValueAsDouble()));
        setpointPositionDegreesPub.set(Units.rotationsToDegrees(pivot.getClosedLoopReference().getValueAsDouble()));
        dutyCyclePub.set(pivot.getDutyCycle().getValueAsDouble());
        atSetpointPub.set(atSetpoint.getAsBoolean());

        SmartDashboard.putBoolean("Stator Current Trigger", statorOverThreshold.getAsBoolean());
    }
}