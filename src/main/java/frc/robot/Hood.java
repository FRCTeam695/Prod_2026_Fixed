package frc.robot;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
    private static final Distance kServoLength = Millimeters.of(100);
    private static final LinearVelocity kMaxServoSpeed = Millimeters.of(20).per(Second);
    private static final double testMaxServoSpeed = 0.22; //0.35 /sec
    private static final double kMinPosition = 0.0; 
    private static final double kMaxPosition = 1.0; //Set this later
    private static final double kPositionTolerance = 0.01;

    private final Servo leftServo;
    private final Servo rightServo;

    private final Servo testServo;

    private double currentPosition = 0.5;
    private double targetPosition = 0.5;
    private double lastUpdateTime = 0;

    public Hood() {
        leftServo = new Servo(1);
        rightServo = new Servo(2);
        testServo = new Servo(0);
        leftServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        rightServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        testServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        setPosition(currentPosition);
        SmartDashboard.putData(this);
    }

    /** Expects a position between 0.0 and 1.0 */
    public void setPosition(double position) {
        final double clampedPosition = MathUtil.clamp(position, kMinPosition, kMaxPosition);
        //leftServo.set(clampedPosition);
        //rightServo.set(clampedPosition);
        testServo.set(clampedPosition);
        targetPosition = clampedPosition;
    }

    /** Expects a position between 0.0 and 1.0 */
    public Command positionCommand(double position) {
        return runOnce(() -> setPosition(position))
            .andThen(Commands.waitUntil(this::isPositionWithinTolerance));
    }

    public boolean isPositionWithinTolerance() {
        return MathUtil.isNear(targetPosition, currentPosition, kPositionTolerance);
    }

    private void updateCurrentPosition() {
        final double currentTime = Timer.getFPGATimestamp();
        final double elapsedTime = currentTime - lastUpdateTime;
        lastUpdateTime = currentTime;

        if (isPositionWithinTolerance()) {
            currentPosition = targetPosition;
            return;
        }

        //final Distance maxDistanceTraveled = kMaxServoSpeed.times(elapsedTime);
        final double maxDistanceTraveled = testMaxServoSpeed * elapsedTime;

        //final double maxPercentageTraveled = maxDistanceTraveled.div(kServoLength).in(Value);
        currentPosition = targetPosition > currentPosition
            ? Math.min(targetPosition, currentPosition + maxDistanceTraveled)
            : Math.max(targetPosition, currentPosition - maxDistanceTraveled);
    }

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable hoodTable = inst.getTable("Hood");

    private final DoublePublisher currentPos = hoodTable.getDoubleTopic("Current Position").publish();
    private final DoublePublisher targetPos = hoodTable.getDoubleTopic("Target Position").publish();

    @Override
    public void periodic() {
        updateCurrentPosition();
        currentPos.set(currentPosition);
        targetPos.set(targetPosition);
    }
}