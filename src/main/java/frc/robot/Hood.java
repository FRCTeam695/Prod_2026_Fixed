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
    private static final double m_MinPosition = 0.0; 
    private static final double m_MaxPosition = 1.0;

    // Maybe kS will differ slightly on different actuators
    private static final double kP = 25;
    private static final double kS = 0.0785;

    // 0-1
    private static final AnalogPotentiometer r_analog = new AnalogPotentiometer(0);

    private final VictorSP r_actuator;
    private final VictorSP l_actuator;

    private double currentPos;
    private double targetPos;
    private double distanceAway = 0;
    private final double actUnitTo_mm = 100;

    private final double A_hoodLen_mm = 168; // pivot arm length (mm)
    private final double B_hypot_mm = 168*Math.sqrt(2); // hypotenuse (mm)
    private final double actLen_mm = 168; // actuator base length (mm)
    private double C_totActLen_mm = actLen_mm + r_analog.get() * actUnitTo_mm; // actuator base length + extension length (mm)

    public Hood() {
        r_actuator = new VictorSP(0);
        l_actuator = new VictorSP(1);
        currentPos = 0; //absolute position encoder
        targetPos = 0;
    }

    /** Expects a position between 0.0 and 1.0 */
    public void setPosition(double unclampedTargetPos) {
        targetPos = MathUtil.clamp(unclampedTargetPos, m_MinPosition, m_MaxPosition);
        currentPos = r_analog.get();
        distanceAway = targetPos - currentPos;

        // manual PID
        double vel = MathUtil.clamp((distanceAway * kP) + kS * Math.signum(distanceAway), -1, 1);
        
        // actuator moves at set velocity
        r_actuator.set(vel);
        l_actuator.set(vel);

        updateActuatorLength(); //mm

        targetPosDeg.set(desiredHoodDeg(targetPos));
    }

    /** Expects a position between 0.0 and 1.0 */
    public Command setActuatorPos(double position) {
        return run(() -> setPosition(position));
    }

    /** Updates total len of actuator + extension length in mm */
    public void updateActuatorLength() {
        C_totActLen_mm = actLen_mm + r_analog.get() * actUnitTo_mm;
    }

    public double currentHoodDeg() {
        return 
        Math.toDegrees(
            Math.acos(
                (Math.pow(A_hoodLen_mm, 2) + Math.pow(B_hypot_mm, 2) - Math.pow(C_totActLen_mm, 2)) 
                / (2*A_hoodLen_mm*B_hypot_mm)
            )
        );
    }

    public double desiredHoodDeg(double targetPos_mm) {
        return 
        Math.toDegrees(
            Math.acos(
                (Math.pow(A_hoodLen_mm, 2) + Math.pow(B_hypot_mm, 2) - Math.pow(targetPos_mm + actLen_mm, 2)) 
                / (2*A_hoodLen_mm*B_hypot_mm)
            )
        );
    }

    public void setDuty(double duty) {
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