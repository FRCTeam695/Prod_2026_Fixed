package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {

    private final Servo r_actuator;
    private final Servo l_actuator;

    private final double min_servo = 0.25; // 0 mm extended
    private final double max_servo = 0.75; // 100 mm extended

    // CHANGE ID
    private final CANcoder r_cancoder = new CANcoder(59);
    private final CANcoder l_cancoder = new CANcoder(58);

    // Constants
    /** pivot arm length (mm) */
    private final double A_HOODLEN = 174.08;

    /** hypotenuse of pivot arm and actuator extended length (mm)*/
    private final double B_HYPOT = 196.85;

    /** actuator base length (mm) */
    private final double ACTLEN = 167.9;

    private final double ACT_TO_MM = 100;

    private ShuffleboardTab hoodTab = Shuffleboard.getTab("Hood");

    // positive numbers bring hood down, negative numbers bring hood up
    private GenericEntry hoodAdjustment = hoodTab.add("Hood Angle Adjustment", 0.0).getEntry();

    private double prevHoodAdjustment = 0.0;
    
    public Hood() {
        r_actuator = new Servo(1);
        l_actuator = new Servo(0);

        CANcoderConfiguration r_config = new CANcoderConfiguration()
            .withMagnetSensor(
                new MagnetSensorConfigs()
                  
                    //      Given a total range of motion less than 1 rotation, users can
                    //  * calculate the discontinuity point using mean(lowerLimit,
                    //  * upperLimit) + 0.5. 
                    //  *

                    .withAbsoluteSensorDiscontinuityPoint(1)
                    .withMagnetOffset(20.0/360 - 0.498047) //0.002441

                    // expected = measured + offset
                    // offset = expected - measured. in this case, expected is 0.2 as we are moving it to 72 degrees
                    // one of the cancoders is flipped
                    .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                    
        
            );
        
        CANcoderConfiguration l_config = new CANcoderConfiguration()
            .withMagnetSensor(
                new MagnetSensorConfigs()
     
                    //      Given a total range of motion less than 1 rotation, users can
                    //  * calculate the discontinuity point using mean(lowerLimit,
                    //  * upperLimit) + 0.5. 
                    //  *

                    .withAbsoluteSensorDiscontinuityPoint(1)
                    .withMagnetOffset(20.0/360 - 0.273438) //-0.002197
                    .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            );

        r_cancoder.getConfigurator().apply(r_config);
        l_cancoder.getConfigurator().apply(l_config);


        // set starting value for cancoder position
        // should ideally be a constant of the retracted position in cancoder rotations
        // r_cancoder.setPosition(0);
        // l_cancoder.setPosition(0);

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
            () -> setPosition(degToActUnit(MathUtil.clamp(deg.getAsDouble() + hoodAdjustment.getDouble(0.0), 52, 72.6)))
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

    public double getHoodExitAngle(CANcoder cancoder){
        return (90 - cancoder.getAbsolutePosition().getValueAsDouble() * 360);
    }
    //(90 - (cancoder.getAbsolutePosition().getValueAsDouble() + offset) * 360) = 70
    //20/360 = canc + offset

    // Networktables
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable hoodTable = inst.getTable("Hood");
    private final DoublePublisher r_currentRot = hoodTable.getDoubleTopic("Right current Position (Rot)").publish();
    private final DoublePublisher l_currentRot = hoodTable.getDoubleTopic("Left current Position (Rot)").publish();


    private final DoublePublisher targetPosDeg = hoodTable.getDoubleTopic("Target deg on hood").publish();


    private final DoublePublisher rightExitAnglePub = hoodTable.getDoubleTopic("Right hood exit angle (deg)").publish();
    private final DoublePublisher leftExitAnglePub = hoodTable.getDoubleTopic("Left hood exit angle (deg)").publish();

    @Override
    public void periodic() {
        double rPos = r_actuator.get();
        double lPos = l_actuator.get();
        r_currentRot.set(rPos);
        l_currentRot.set(lPos);

        rightExitAnglePub.set(getHoodExitAngle(r_cancoder));
        leftExitAnglePub.set(getHoodExitAngle(l_cancoder));

    }
}