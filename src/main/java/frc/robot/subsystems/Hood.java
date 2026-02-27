package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {

    /** Actuonix L16 100mm extension length 35mm/s */
    private final VictorSP r_actuator;
    /** Actuonix L16 100mm extension length 35mm/s */
    private final VictorSP l_actuator;

    /** gets actuator position from 0 to 1*/
    private static final AnalogPotentiometer r_analog = new AnalogPotentiometer(0);
    private static final AnalogPotentiometer l_analog = new AnalogPotentiometer(1);

    // Constants
    /** pivot arm length (mm) */
    private final double A_HOODLEN = 174.08;

    /** hypotenuse of pivot arm and actuator extended length (mm)*/
    private final double B_HYPOT = 196.85;

    /** actuator base length (mm) */
    private final double ACTLEN = 167.9;


    private final double ACT_TO_MM = 100;

    // Maybe kS will differ slightly on different actuators
    private final double kS = 0.18;
    private final double kP = 12.5;

    public Hood() {
        r_actuator = new VictorSP(0);
        l_actuator = new VictorSP(1);
    }

    /** Expects a position between 0.0 and 1.0 */
    private void setPosition(double unclampedTargetPos) {

        double targetPos = MathUtil.clamp(unclampedTargetPos, 0, 1);
        SmartDashboard.putNumber("target position", targetPos);
        double currentPos = r_analog.get(); //absolute position encoder
        double targetDiff = targetPos - currentPos;
        double actDiff = r_analog.get() - l_analog.get();

        // manual PID
        double r_vel = MathUtil.clamp((targetDiff * kP) + kS * Math.signum(targetDiff), -1, 1);
        double l_vel = MathUtil.clamp((actDiff * kP) + kS * Math.signum(actDiff), -1, 1);

        SmartDashboard.putNumber("R_VEL", r_vel);
        SmartDashboard.putNumber("L_VEL", l_vel)
;
        
        // actuator moves at set velocity
        r_actuator.set(r_vel);
        l_actuator.set(l_vel);

        // networktables
        targetPosDeg.set(actUnitToDeg(targetPos));
    }

    /**  */
    public Command setActuatorDeg(DoubleSupplier deg) {
        return run(
            () -> setPosition(degToActUnit(MathUtil.clamp(deg.getAsDouble(), 45, 72.1)))
        );
    }


    public double actUnitToDeg(double actUnit){
        double C = ACTLEN + (actUnit * ACT_TO_MM);
        double theta2 = Math.acos(
                (Math.pow(A_HOODLEN, 2) + Math.pow(B_HYPOT, 2) - Math.pow(C, 2)) / (2*A_HOODLEN*B_HYPOT)
            ) - Math.toRadians(36);
        double dy = A_HOODLEN * Math.sin(theta2);
        double dx = A_HOODLEN * Math.cos(theta2);
        return Math.toDegrees(Math.atan2(dx, dy));
    }


public double degToActUnit(double deg) {
    double theta2 = Math.toRadians(90.0 - deg);
    double phi = theta2 + Math.toRadians(36.0);
    double cSquared = Math.pow(A_HOODLEN, 2) + Math.pow(B_HYPOT, 2) 
                    - (2 * A_HOODLEN * B_HYPOT * Math.cos(phi));
    
    double c = Math.sqrt(cSquared);

    return (c - ACTLEN) / ACT_TO_MM;
}

    public void setDuty(double duty) {
        r_actuator.set(duty);
        l_actuator.set(duty);
    }

    public Command stop(){
        return runOnce(() -> setDuty(0));
    }

    // Networktables
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable hoodTable = inst.getTable("Hood");
    private final DoublePublisher r_currentRot = hoodTable.getDoubleTopic("Right current Position (Rot)").publish();
    private final DoublePublisher l_currentRot = hoodTable.getDoubleTopic("Left current Position (Rot)").publish();

    private final DoublePublisher currentHoodDeg = hoodTable.getDoubleTopic("Current deg on hood").publish();
    private final DoublePublisher targetPosDeg = hoodTable.getDoubleTopic("Target deg on hood").publish();
    private final DoublePublisher r_voltageApplied = hoodTable.getDoubleTopic("Voltage applied r").publish();
    private final DoublePublisher l_voltageApplied = hoodTable.getDoubleTopic("Voltage applied l").publish();

    @Override
    public void periodic() {
        r_currentRot.set(r_analog.get());
        l_currentRot.set(l_analog.get());
        currentHoodDeg.set(actUnitToDeg(r_analog.get()));
        r_voltageApplied.set(r_actuator.getVoltage());
        l_voltageApplied.set(l_actuator.getVoltage());
        SmartDashboard.putNumber("Actuator duty cycle", r_actuator.get());
    }
}