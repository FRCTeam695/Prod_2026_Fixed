package frc.BisonLib.BaseProject.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;


import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;


import org.apache.commons.math3.analysis.interpolation.BicubicInterpolatingFunction;
import org.apache.commons.math3.analysis.interpolation.BicubicInterpolator;

public class ShootOnTheMoveHelper {
    private BicubicInterpolatingFunction rpmSpline;
    private BicubicInterpolatingFunction angleSpline;
    private BicubicInterpolatingFunction timeSpline;


    private Supplier<Pose2d> robotPoseSupplier;
    private Supplier<ChassisSpeeds> robotChassisSpeedsSupplier;

    public ShootOnTheMoveHelper(String csvFileName, Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> robotChassisSpeedsSupplier) {
        loadCSV(csvFileName);
        this.robotPoseSupplier = robotPoseSupplier;
        this.robotChassisSpeedsSupplier = robotChassisSpeedsSupplier;
    }

    private void loadCSV(String fileName) {
        File file = new File(Filesystem.getDeployDirectory(), fileName);
        List<Double> distList = new ArrayList<>();
        List<Double> velList = new ArrayList<>();
        List<Double> timeList = new ArrayList<>();
        List<Double[]> dataRows = new ArrayList<>();

        try (BufferedReader br = new BufferedReader(new FileReader(file))) {
            String line = br.readLine(); // Skip header: distance, robot_vel, angle, rpm, cost
            
            while ((line = br.readLine()) != null) {
                String[] values = line.split(",");
                double d = Double.parseDouble(values[0]);
                double v = Double.parseDouble(values[1]);
                double angle = Double.parseDouble(values[2]);
                double rpm = Double.parseDouble(values[3]);
                double time = Double.parseDouble(values[4]);

                if (!distList.contains(d)) distList.add(d);
                if (!velList.contains(v)) velList.add(v);
                if (!timeList.contains(time)) timeList.add(time);
                
                dataRows.add(new Double[]{d, v, angle, rpm, time});
            }

            // Apache requires sorted, unique arrays for the grid axes
            double[] xArr = distList.stream().sorted().mapToDouble(Double::doubleValue).toArray();
            double[] yArr = velList.stream().sorted().mapToDouble(Double::doubleValue).toArray();

            double[][] angleGrid = new double[xArr.length][yArr.length];
            double[][] rpmGrid = new double[xArr.length][yArr.length];
            double[][] timeGrid = new double[xArr.length][yArr.length];

            // Map the flat list into the 2D grid
            for (Double[] row : dataRows) {
                int xIdx = binarySearch(xArr, row[0]);
                int yIdx = binarySearch(yArr, row[1]);
                angleGrid[xIdx][yIdx] = row[2];
                rpmGrid[xIdx][yIdx] = row[3];
                timeGrid[xIdx][yIdx] = row[4];
            }

            BicubicInterpolator interpolator = new BicubicInterpolator();
            this.angleSpline = interpolator.interpolate(xArr, yArr, angleGrid);
            this.rpmSpline = interpolator.interpolate(xArr, yArr, rpmGrid);
            this.timeSpline = interpolator.interpolate(xArr, yArr, timeGrid);

        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public ShooterSetpoint getSetpoint() {
        final Transform2d robotToShooter = new Transform2d(0.254, 0.0, new Rotation2d());

        ChassisSpeeds chassisSpeeds = robotChassisSpeedsSupplier.get();
        Pose2d robotPose = robotPoseSupplier.get();

 
        // accounts for extra shooter vel if we are spinning
        double shooterVx = chassisSpeeds.vxMetersPerSecond - 
                          (chassisSpeeds.omegaRadiansPerSecond * robotToShooter.getY());
        double shooterVy = chassisSpeeds.vyMetersPerSecond + 
                          (chassisSpeeds.omegaRadiansPerSecond * robotToShooter.getX());

        Translation2d shooterVelocity = new Translation2d(shooterVx, shooterVy);

        // phase delay compensation
        Translation2d phaseDelayComp = shooterVelocity.times(0.03); // 20ms latency
        Pose2d shooterPose = robotPose.transformBy(robotToShooter)
                                      .plus(new Transform2d(phaseDelayComp.getX(), phaseDelayComp.getY(), new Rotation2d()));

        Pose2d hub = isRedAlliance() ? Constants.FieldConstants.Red.hub : Constants.FieldConstants.Blue.hub;
        
        Translation2d shooterToHub = hub.getTranslation().minus(shooterPose.getTranslation());

        //seed ToF
        Rotation2d angleToHub = shooterToHub.getAngle();
        double virtualVr = (shooterVelocity.getX() * angleToHub.getCos()) + 
                           (shooterVelocity.getY() * angleToHub.getSin());

        double tof = timeSpline.value(shooterToHub.getNorm(), virtualVr); 
        double virtualDist = shooterToHub.getNorm();
        
        Translation2d ghostFieldPos = hub.getTranslation(); 

        for (int i = 0; i < 20; i++) {
            // calculate virtual hub pos
            ghostFieldPos = hub.getTranslation().minus(shooterVelocity.times(tof));
            
            // recalculate geometry to this new ghost point
            Translation2d ghostVector = ghostFieldPos.minus(shooterPose.getTranslation());
            virtualDist = ghostVector.getNorm();
            Rotation2d targetAngle = ghostVector.getAngle();

            // project velocity onto the new angle
            virtualVr = (shooterVelocity.getX() * targetAngle.getCos()) + 
                        (shooterVelocity.getY() * targetAngle.getSin());
            
            double newtof = timeSpline.value(virtualDist, virtualVr);
            
            if(Math.abs(tof - newtof) / newtof < 0.01){ // <1% error then we can stop
                tof = newtof;
                break;
            }
            tof = newtof;
        }


        return new ShooterSetpoint(
            angleSpline.value(virtualDist, virtualVr),
            rpmSpline.value(virtualDist, virtualVr),
            ghostFieldPos
        );
    }


    private int binarySearch(double[] arr, double target) {
        int idx = java.util.Arrays.binarySearch(arr, target);
        return idx < 0 ? -(idx + 1) : idx;
    }


    public boolean isRedAlliance(){
        var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                boolean temp = (alliance.get() == DriverStation.Alliance.Red) ? true : false;
                return temp;
              }
        return false;
    }


    

    public record ShooterSetpoint(double angle, double rpm, Translation2d virtualTarget) {}
}