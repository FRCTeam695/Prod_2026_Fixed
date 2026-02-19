package frc.robot;

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

    /** extended full actuator length (mm) */
    private double C_ActExtended; 

    private final double ACT_TO_MM = 100;

    // Maybe kS will differ slightly on different actuators
    private final double kS = 0.18;
    private final double kP = 12.5;

    public Hood() {
        r_actuator = new VictorSP(0);
        l_actuator = new VictorSP(1);

        updateActuatorLength();
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
        
        // actuator moves at set velocity
        r_actuator.set(r_vel);
        l_actuator.set(l_vel);

        // networktables
        targetPosDeg.set(hoodDeg("desired", targetPos));
    }

    /**  */
    public Command setActuatorDeg(double deg) {
        return run(() -> setPosition(degToActUnit(deg + hoodDeg("original hood deg", -1))));
    }

    /** Updates total len of actuator + extension length in mm */
    public void updateActuatorLength() {
        C_ActExtended = ACTLEN + r_analog.get() * ACT_TO_MM;
    }

    public double hoodDeg(String type, double targetPos_mm) {
        double temp = 0;

        // switches which angle we're calculating
        switch (type) {
            case "current":
                temp = C_ActExtended;
                break;
            case "desired":
                temp = targetPos_mm + ACTLEN;
                break;
            default:
                temp = ACTLEN;
        }

        return Math.toDegrees(Math.acos(
                (Math.pow(A_HOODLEN, 2) + Math.pow(B_HYPOT, 2) - Math.pow(temp, 2)) / (2*A_HOODLEN*B_HYPOT)
            )
        );
    }

    public double degToActUnit(double deg) {
        return (Math.sqrt(
                Math.pow(A_HOODLEN, 2) + Math.pow(B_HYPOT, 2) - (2*A_HOODLEN*B_HYPOT*Math.cos(Math.toRadians(deg)))
            ) - ACTLEN
        ) / ACT_TO_MM;
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
        updateActuatorLength(); //mm
        r_currentRot.set(r_analog.get());
        l_currentRot.set(l_analog.get());
        currentHoodDeg.set(hoodDeg("current", -1));
        r_voltageApplied.set(r_actuator.getVoltage());
        l_voltageApplied.set(l_actuator.getVoltage());
        SmartDashboard.putNumber("Actuator duty cycle", r_actuator.get());
    }
}