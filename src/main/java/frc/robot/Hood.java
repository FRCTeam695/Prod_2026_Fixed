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
    private static final double m_PositionTolerance = 0.01;

    private static final AnalogPotentiometer m_analog = new AnalogPotentiometer(0);
    // 0 - 1
    //3571, -2.746p

    private final VictorSP actuator;

    private double currentPosition;
    private double targetPosition;

    private final double A = 168; // pivot arm length (mm)
    private final double B = 168*Math.sqrt(2); // hypotenuse (mm)
    private final double actLen = 168; // actuator base length (mm)
    private double C = actLen + m_analog.get() * 100; // actuator base length + extension length (mm)

    private final double radToDeg = 180/(Math.PI);
    private double hoodDeg = Math.acos((A*A + B*B - C*C) / (2*A*B)) * radToDeg;

    public Hood() {
        actuator = new VictorSP(0);
        currentPosition = 0;//absolute position encoder
        targetPosition = 0;
    }

    /** Expects a position between 0.0 and 1.0 */
    public void setPosition(double desiredPosition) {
        targetPosition = MathUtil.clamp(desiredPosition, m_MinPosition, m_MaxPosition);
        currentPosition = m_analog.get();
        if (targetPosition < currentPosition)
            actuator.set(-1);
        else if (targetPosition > currentPosition)
            actuator.set(1.0);
        else
            actuator.set(0.0);
    }

    /** Expects a position between 0.0 and 1.0 */
    public Command positionCommand(double position) {
        return run(() -> setPosition(position))
            .until(this::isPositionWithinTolerance)
            .andThen(runOnce(()->actuator.stopMotor()));
    }

    public boolean isPositionWithinTolerance() {
        return MathUtil.isNear(targetPosition, currentPosition, m_PositionTolerance);
    }

    public void setDuty(double duty) {
        actuator.set(duty);
    }

    public Command stop(){
        return runOnce(() -> setDuty(0));
    }

    // Networktables
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable hoodTable = inst.getTable("Hood");
    private final DoublePublisher currentPosRot = hoodTable.getDoubleTopic("Current Position (Rot)").publish();
    private final DoublePublisher currentPosFinalDeg = hoodTable.getDoubleTopic("Final deg on hood").publish();
    private final DoublePublisher targetPos = hoodTable.getDoubleTopic("Target Position").publish();

    @Override
    public void periodic() {
        C = actLen + m_analog.get() * 100; //mm
        hoodDeg = Math.acos((A*A + B*B - C*C) / (2*A*B)) * radToDeg; //all measurements in mm
        currentPosRot.set(m_analog.get());
        currentPosFinalDeg.set(hoodDeg);
        targetPos.set(targetPosition);
        SmartDashboard.putNumber("Actuator duty cycle", actuator.get());
    }
}