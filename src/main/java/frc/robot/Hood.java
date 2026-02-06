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

    private double currentPosition;
    private double targetPosition;

    private final double A = 168; // pivot arm length (mm)
    private final double B = 168*Math.sqrt(2); // hypotenuse (mm)
    private final double actLen = 168; // actuator base length (mm)
    private double C = actLen + r_analog.get() * 100; // actuator base length + extension length (mm)

    private final double radToDeg = 180/(Math.PI);

    public Hood() {
        r_actuator = new VictorSP(0);
        l_actuator = new VictorSP(1);
        currentPosition = 0;//absolute position encoder
        targetPosition = 0;
    }

    /** Expects a position between 0.0 and 1.0 */
    public void setPosition(double desiredPosition) {
        targetPosition = MathUtil.clamp(desiredPosition, m_MinPosition, m_MaxPosition);
        currentPosition = r_analog.get();

        double diff = targetPosition - currentPosition;
        double vel = MathUtil.clamp((diff * kP) + kS * Math.signum(diff), -1, 1);
        
        r_actuator.set(vel);
        l_actuator.set(vel);

        C = actLen + r_analog.get() * 100; //mm

        double extensionLength = desiredPosition * 100;
        targetPosDeg.set(desiredHoodDeg(extensionLength));
    }

    /** Expects a position between 0.0 and 1.0 */
    public Command positionCommand(double position) {
        return run(() -> setPosition(position));
    }

    public double currentHoodDeg() {
        return Math.acos((A*A + B*B - C*C) / (2*A*B)) * radToDeg;
    }

    public double desiredHoodDeg(double extensionLength) {
        return Math.acos((A*A + B*B - Math.pow(extensionLength+actLen, 2)) / (2*A*B)) * radToDeg;
    }

    // Testing actuator
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