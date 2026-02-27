package frc.BisonLib.BaseProject.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.Supplier;



public class SOTMSetpointGenerator extends SubsystemBase{

    private InterpolatingDoubleTreeMap rpmMap;
    private InterpolatingDoubleTreeMap angleMap;
    private InterpolatingDoubleTreeMap feederMap;
    private InterpolatingDoubleTreeMap timeMap;

    private Supplier<Pose2d> robotPoseSupplier;
    private ShooterSetpoint cachedSetpoint;

    public SOTMSetpointGenerator(Supplier<Pose2d> robotPoseSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;

        rpmMap = new InterpolatingDoubleTreeMap();
        angleMap = new InterpolatingDoubleTreeMap();
        timeMap = new InterpolatingDoubleTreeMap();
        feederMap = new InterpolatingDoubleTreeMap();

        // 0.97, 4.13
        // 2.14, 4.09 :ll poses

        /*
         * This is a new thread for Feb 24th shot testing videos.  Shot settings include:
            d, a, s, f, q
            d = distance from face of hub to front of aluminum frame (meters)
            a = shooter hood angle (degrees)
            s = shooter speed (percent)
            f = feed rate (meters per second)
            q = quantity of balls shot (each)

            far: 2.6, 60, 65, 0.5, 26/30 (4 didn't leave the intake)
            close: 1.5, 65.17, 60, 1, 27/30 (3 stayed in hopper)
         */
        addDatapointToTable(
            Constants.FieldConstants.Blue.hub.getTranslation().getDistance(new Translation2d(0.2, 3.935)), 
            2 * Math.PI * Units.inchesToMeters(2) * 100 * 0.68,
            59,
            -0.5, 
            1.36);
        addDatapointToTable(
            Constants.FieldConstants.Blue.hub.getTranslation().getDistance(new Translation2d(0.90, 4.00)), 
            2 * Math.PI * Units.inchesToMeters(2) * 100 * 0.63,
            60,
            -0.5, 
            1.36);
        addDatapointToTable(
            Constants.FieldConstants.Blue.hub.getTranslation().getDistance(new Translation2d(2.125, 3.98)), 
            19.1511488163, 
            65.17,
            -1.0, 
            1.3);
        addDatapointToTable(
            Constants.FieldConstants.Blue.hub.getTranslation().getDistance(new Translation2d(3.73, 4.11)), 
            2 * Math.PI * Units.inchesToMeters(2) * 100 * 0.60,
            71,
            -1.0, 
            1.36);
    }

    private void addDatapointToTable(double distance, double shotSpeed, double hoodAngle, double feederSpeed, double shotTime){
        rpmMap.put(distance, shotSpeed);
        angleMap.put(distance, hoodAngle);
        feederMap.put(distance, feederSpeed);
        timeMap.put(distance, shotTime);
    }

    private ShooterSetpoint getSetpoint() {
        Pose2d robotPose = robotPoseSupplier.get();

        Pose2d hub = isRedAlliance() ? Constants.FieldConstants.Red.hub : Constants.FieldConstants.Blue.hub;
        
        double dist = robotPose.getTranslation().getDistance(hub.getTranslation());

        SmartDashboard.putNumber("Dist", dist);
        return new ShooterSetpoint(
            angleMap.get(dist),
            rpmMap.get(dist),
            feederMap.get(dist),
            hub.getTranslation()
        );
    }



    public boolean isRedAlliance(){
        var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                boolean temp = (alliance.get() == DriverStation.Alliance.Red) ? true : false;
                return temp;
              }
        return false;
    }

    public ShooterSetpoint getCachedSetpoint(){
        if(cachedSetpoint != null){
            SmartDashboard.putBoolean("updating cached setpoint", false);
            return cachedSetpoint;
        }
        SmartDashboard.putBoolean("updating cached setpoint", true);
        cachedSetpoint = getSetpoint();
        return cachedSetpoint;
    }

    public void clearCachedSetpoint(){
        if( cachedSetpoint != null){
            SmartDashboard.putNumber("Cached angle", cachedSetpoint.angle);
            SmartDashboard.putNumber("Cached rpm", cachedSetpoint.rpm);
            SmartDashboard.putNumber("Cached feed speed", cachedSetpoint.feedSpeed);
            SmartDashboard.putString("Cached Virtual Target", cachedSetpoint.virtualTarget.toString());
        }
        cachedSetpoint = null;
    }


    

    public record ShooterSetpoint(double angle, double rpm, double feedSpeed, Translation2d virtualTarget) {}
}