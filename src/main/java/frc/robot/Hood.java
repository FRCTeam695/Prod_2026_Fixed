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

    // Constants
    /** pivot arm length (mm) */
    private final double A_HOODLEN = 168;

    /** hypotenuse of pivot arm and actuator extended length (mm)*/
    private final double B_HYPOT = 168 * Math.sqrt(2);

    /** actuator base length (mm) */
    private final double ACTLEN = 168;

    /** extended full actuator length (mm) */
    private double C_ActExtended; 

    private final double ACTMINPOS = 0.0; 
    private final double ACTMAXPOS = 1.0;
    private final double ACT_TO_MM = 100;

    // Maybe kS will differ slightly on different actuators
    private final double kS = 0.0785;
    private final double kP = 25;

    public Hood() {
        r_actuator = new VictorSP(0);
        l_actuator = new VictorSP(1);

        updateActuatorLength();
    }

    /** Expects a position between 0.0 and 1.0 */
    private void setPosition(double unclampedTargetPos) {

        double targetPos = MathUtil.clamp(unclampedTargetPos, ACTMINPOS, ACTMAXPOS);
        double currentPos = r_analog.get(); //absolute position encoder
        double targetDistance = targetPos - currentPos;
        //double actDiff = r_analog;

        // manual PID
        double r_vel = MathUtil.clamp((targetDistance * kP) + kS * Math.signum(targetDistance), -1, 1);
        double l_vel = MathUtil.clamp((targetDistance * kP) + kS * Math.signum(targetDistance), -1, 1);
        
        // actuator moves at set velocity
        r_actuator.set(r_vel);
        l_actuator.set(l_vel);

        updateActuatorLength(); //mm

        targetPosDeg.set(desiredHoodDeg(targetPos));
    }

    /** Expects a position between 0.0 and 1.0 */
    public Command setActuatorPos(double position) {
        return run(() -> setPosition(position));
    }

    /** Updates total len of actuator + extension length in mm */
    public void updateActuatorLength() {
        C_ActExtended = ACTLEN + r_analog.get() * ACT_TO_MM;
    }

    public double currentHoodDeg() {
        return 
        Math.toDegrees(
            Math.acos(
                (Math.pow(A_HOODLEN, 2) + Math.pow(B_HYPOT, 2) - Math.pow(C_ActExtended, 2)) 
                / (2*A_HOODLEN*B_HYPOT)
            )
        );
    }

    public double desiredHoodDeg(double targetPos_mm) {
        return 
        Math.toDegrees(
            Math.acos(
                (Math.pow(A_HOODLEN, 2) + Math.pow(B_HYPOT, 2) - Math.pow(targetPos_mm + ACTLEN, 2)) 
                / (2*A_HOODLEN*B_HYPOT)
            )
        );
    }

    private void setDuty(double duty) {
        r_actuator.set(duty);
    }

    public Command stop(){
        return runOnce(() -> setDuty(0));
    }

    // Networktables
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable hoodTable = inst.getTable("Hood");
    private final DoublePublisher currentPosRot = hoodTable.getDoubleTopic("Current Position (Rot)").publish();
    private final DoublePublisher currentPosDeg = hoodTable.getDoubleTopic("Current deg on hood").publish();
    private final DoublePublisher targetPosDeg = hoodTable.getDoubleTopic("Target deg on hood").publish();

    @Override
    public void periodic() {
        currentPosRot.set(r_analog.get());
        currentPosDeg.set(currentHoodDeg());
        SmartDashboard.putNumber("Actuator duty cycle", r_actuator.get());
    }
}