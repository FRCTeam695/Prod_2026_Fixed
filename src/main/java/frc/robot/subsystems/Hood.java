package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {

    private final Servo r_actuator;
    private final Servo l_actuator;

    private final double min_servo = 0.25; // 0 mm extended
    private final double max_servo = 0.75; // 100 mm extended

    // Constants
    /** pivot arm length (mm) */
    private final double A_HOODLEN = 174.08;

    /** hypotenuse of pivot arm and actuator extended length (mm)*/
    private final double B_HYPOT = 196.85;

    /** actuator base length (mm) */
    private final double ACTLEN = 167.9;


    private final double ACT_TO_MM = 100;

    public Hood() {
        r_actuator = new Servo(1);
        l_actuator = new Servo(0);
    }

    /** Expects a position between 0.0 and 1.0 */
    private void setPosition(double unclampedTargetPos) {
        // networktables
        targetPosDeg.set(actUnitToDeg(unclampedTargetPos));

        unclampedTargetPos = min_servo + unclampedTargetPos * (max_servo - min_servo); //change position into servo units

        
        // actuator moves at set velocity
        r_actuator.set(unclampedTargetPos);
        l_actuator.set(unclampedTargetPos);
    }

    /**  */
    public Command setActuatorDeg(DoubleSupplier deg) {
        return run(
            () -> setPosition(degToActUnit(MathUtil.clamp(deg.getAsDouble(), 52, 72.1)))
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

    // Networktables
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable hoodTable = inst.getTable("Hood");
    private final DoublePublisher r_currentRot = hoodTable.getDoubleTopic("Right current Position (Rot)").publish();
    private final DoublePublisher l_currentRot = hoodTable.getDoubleTopic("Left current Position (Rot)").publish();

    private final DoublePublisher currentHoodDegRight = hoodTable.getDoubleTopic("Right current deg on hood").publish();
    private final DoublePublisher currentHoodDegLeft = hoodTable.getDoubleTopic("Left current deg on hood").publish();

    private final DoublePublisher targetPosDeg = hoodTable.getDoubleTopic("Target deg on hood").publish();

    @Override
    public void periodic() {
        double rPos = r_actuator.get();
        double lPos = l_actuator.get();
        r_currentRot.set(rPos);
        l_currentRot.set(lPos);

        currentHoodDegRight.set(actUnitToDeg(rPos));
        currentHoodDegLeft.set(actUnitToDeg(lPos));
    }
}