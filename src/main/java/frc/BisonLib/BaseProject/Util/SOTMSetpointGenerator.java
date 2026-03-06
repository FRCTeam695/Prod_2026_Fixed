package frc.BisonLib.BaseProject.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    private Supplier<ChassisSpeeds> robotSpeedsSupplier;
    private ShotParameters cachedSetpoint;

    public SOTMSetpointGenerator(Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> robotSpeedsSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.robotSpeedsSupplier = robotSpeedsSupplier;

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

         // this table is furthest to closest location
        addDatapointToTable(
            Constants.FieldConstants.Blue.HUB.getTranslation().getDistance(new Translation2d(0.2, 3.935)), 
            2 * Math.PI * Units.inchesToMeters(2) * 100 * 0.68,
            59,
            -0.5, 
            1.42);
        addDatapointToTable(
            Constants.FieldConstants.Blue.HUB.getTranslation().getDistance(new Translation2d(0.90, 4.00)), 
            2 * Math.PI * Units.inchesToMeters(2) * 100 * 0.63,
            60,
            -0.5, 
            1.36);
        addDatapointToTable(
            Constants.FieldConstants.Blue.HUB.getTranslation().getDistance(new Translation2d(2.125, 3.98)), 
            19.1511488163, 
            65.17,
            -1.0, 
            1.3);
        addDatapointToTable(
            Constants.FieldConstants.Blue.HUB.getTranslation().getDistance(new Translation2d(3.73, 4.11)), 
            2 * Math.PI * Units.inchesToMeters(2) * 100 * 0.60,
            71,
            -1.0, 
            1.3);
    }

    private void addDatapointToTable(double distance, double shotSpeed, double hoodAngle, double feederSpeed, double shotTime){
        rpmMap.put(distance, shotSpeed);
        angleMap.put(distance, hoodAngle);
        feederMap.put(distance, feederSpeed);
        timeMap.put(distance, shotTime);
    }

    private ShotParameters getSetpoint() {
        Pose2d robotPose = robotPoseSupplier.get();

        Pose2d hub = isRedAlliance() ? Constants.FieldConstants.Red.HUB : Constants.FieldConstants.Blue.HUB;
        
        double dist = robotPose.getTranslation().getDistance(hub.getTranslation());

        SmartDashboard.putNumber("Dist", dist);
        return new ShotParameters(
            angleMap.get(dist),
            rpmMap.get(dist),
            feederMap.get(dist),
            hub.getTranslation()
        );
    }

    public ShotParameters getTOFSetpoint(){
        Pose2d robotPose = robotPoseSupplier.get();
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeedsSupplier.get(), robotPose.getRotation());
        final Transform2d robotToShooter = new Transform2d(-Units.inchesToMeters(1.125), 0.0, new Rotation2d());

 
        // accounts for extra shooter vel if we are spinning
        double shooterVx = chassisSpeeds.vxMetersPerSecond - 
                          (chassisSpeeds.omegaRadiansPerSecond * robotToShooter.getY());
        double shooterVy = chassisSpeeds.vyMetersPerSecond + 
                          (chassisSpeeds.omegaRadiansPerSecond * robotToShooter.getX());
                          

        Translation2d shooterVelocity = new Translation2d(shooterVx, shooterVy);

        // phase delay compensation
        Translation2d phaseDelayComp = shooterVelocity.times(0.03); // 30ms latency
        Pose2d shooterPose = robotPose.transformBy(robotToShooter)
                                      .plus(new Transform2d(phaseDelayComp.getX(), phaseDelayComp.getY(), new Rotation2d()));

        Pose2d hub = isRedAlliance() ? Constants.FieldConstants.Red.HUB : Constants.FieldConstants.Blue.HUB;

        Translation2d shooterToHub = hub.getTranslation().minus(shooterPose.getTranslation());

        double tof = timeMap.get(shooterToHub.getNorm()); 
        double virtualDist = shooterToHub.getNorm();
        
        Translation2d ghostFieldPos = hub.getTranslation(); 

        for (int i = 0; i < 20; i++) {
            // calculate virtual hub pos
            ghostFieldPos = hub.getTranslation().minus(shooterVelocity.times(tof));
            
            // recalculate geometry to this new ghost point
            Translation2d ghostVector = ghostFieldPos.minus(shooterPose.getTranslation());
            virtualDist = ghostVector.getNorm();

            double newtof = timeMap.get(virtualDist);
            
            if(Math.abs(tof - newtof) / newtof < 0.01){ // <1% error then we can stop
                tof = newtof;
                break;
            }
            tof = newtof;
        }

        return new ShotParameters(
            angleMap.get(virtualDist),
            rpmMap.get(virtualDist),
            feederMap.get(virtualDist),
            ghostFieldPos
        );
    }

    public ShotParameters getTOFSetpointVaccounting(){
        Pose2d robotPose = robotPoseSupplier.get();
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeedsSupplier.get(), robotPose.getRotation());;
        final Transform2d robotToShooter = new Transform2d(-Units.inchesToMeters(1.125 + 19), 0.0, new Rotation2d());

        // accounts for extra shooter vel if we are spinning
        double shooterVx = chassisSpeeds.vxMetersPerSecond - 
                          (chassisSpeeds.omegaRadiansPerSecond * robotToShooter.getY());
        double shooterVy = chassisSpeeds.vyMetersPerSecond + 
                          (chassisSpeeds.omegaRadiansPerSecond * robotToShooter.getX());
                          
        Translation2d shooterVelocity = new Translation2d(shooterVx, shooterVy);

        // phase delay compensation
        Translation2d phaseDelayComp = shooterVelocity.times(0.03); // 30ms latency
        Pose2d shooterPose = robotPose.transformBy(robotToShooter)
                                      .plus(new Transform2d(phaseDelayComp.getX(), phaseDelayComp.getY(), new Rotation2d()));

        Pose2d hub = isRedAlliance() ? Constants.FieldConstants.Red.HUB : Constants.FieldConstants.Blue.HUB;

        Translation2d shooterToHub = hub.getTranslation().minus(shooterPose.getTranslation());

        // seed ToF
        Rotation2d angleToHub = shooterToHub.getAngle();
        double virtualVr = (shooterVelocity.getX() * angleToHub.getCos()) + 
                           (shooterVelocity.getY() * angleToHub.getSin());

        double tof = timeMap.get(shooterToHub.getNorm()); 
        
        Translation2d ghostFieldPos = hub.getTranslation(); 
        double mapDist = robotPose.getTranslation().getDistance(hub.getTranslation()); // Initial map dist

        for (int i = 0; i < 20; i++) {
            // calculate virtual hub pos based on current ToF
            ghostFieldPos = hub.getTranslation().minus(shooterVelocity.times(tof));
            
            // 1. SHOOTER GEOMETRY (For Physics)
            Translation2d shooterGhostVector = ghostFieldPos.minus(shooterPose.getTranslation());
            double shooterDist = shooterGhostVector.getNorm();
            Rotation2d targetAngle = shooterGhostVector.getAngle();

            // 2. ROBOT CENTER GEOMETRY (For Map Lookups)
            Translation2d robotGhostVector = ghostFieldPos.minus(robotPose.getTranslation());
            mapDist = robotGhostVector.getNorm();

            // Project velocity onto the new angle
            virtualVr = (shooterVelocity.getX() * targetAngle.getCos()) + 
                        (shooterVelocity.getY() * targetAngle.getSin());
            
            // Look up the baseline TOF using the ROBOT CENTER distance
            double baseTof = timeMap.get(mapDist);
            
            // Calculate effective velocity using the physical SHOOTER distance
            double baseVelocity = shooterDist / baseTof;
            double effectiveVelocity = baseVelocity + virtualVr;
            
            // Calculate new TOF
            double newtof = (effectiveVelocity > 0.05) ? (shooterDist / effectiveVelocity) : baseTof;
            
            if(Math.abs(tof - newtof) / newtof < 0.01){ // <1% error then we can stop
                tof = newtof;
                break;
            }
            tof = newtof;
        }

        // Return the setpoints using the ROBOT CENTER distance, because that's how they were tuned!
        return new ShotParameters(
            angleMap.get(mapDist),
            rpmMap.get(mapDist),
            feederMap.get(mapDist),
            ghostFieldPos 
        );
    }

    public Translation2d getClosestStockpileTarget(){
        Pose2d robotPose = robotPoseSupplier.get();

        Translation2d stockpileLeft = isRedAlliance() ? Constants.FieldConstants.Red.LEFT_STOCKPILE : Constants.FieldConstants.Blue.LEFT_STOCKPILE;
        Translation2d stockpileRight = isRedAlliance() ? Constants.FieldConstants.Red.RIGHT_STOCKPILE : Constants.FieldConstants.Blue.RIGHT_STOCKPILE;
        
        if (robotPose.getTranslation().getDistance(stockpileRight) < robotPose.getTranslation().getDistance(stockpileLeft)){
            return stockpileRight;
        }
        return stockpileLeft;
    }



    public boolean isRedAlliance(){
        var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                boolean temp = (alliance.get() == DriverStation.Alliance.Red) ? true : false;
                return temp;
              }
        return false;
    }

    public ShotParameters getCachedSetpoint(){
        if(cachedSetpoint != null){
            // SmartDashboard.putBoolean("updating cached setpoint", false);
            return cachedSetpoint;
        }
        // SmartDashboard.putBoolean("updating cached setpoint", true);
        cachedSetpoint = getTOFSetpoint();
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


    

    public record ShotParameters(double angle, double rpm, double feedSpeed, Translation2d virtualTarget) {}
}