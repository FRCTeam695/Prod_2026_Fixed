package frc.BisonLib.BaseProject.Util;

import edu.wpi.first.wpilibj.Filesystem;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.apache.commons.math3.analysis.interpolation.BicubicInterpolatingFunction;
import org.apache.commons.math3.analysis.interpolation.BicubicInterpolator;

public class ShooterInterpolationMap {
    private BicubicInterpolatingFunction rpmSpline;
    private BicubicInterpolatingFunction angleSpline;

    private double minDistance, maxDistance;
    private double minVel, maxVel;

    public ShooterInterpolationMap(String csvFileName) {
        loadCSV(csvFileName);
    }

    private void loadCSV(String fileName) {
        File file = new File(Filesystem.getDeployDirectory(), fileName);
        List<Double> distList = new ArrayList<>();
        List<Double> velList = new ArrayList<>();
        List<Double[]> dataRows = new ArrayList<>();

        try (BufferedReader br = new BufferedReader(new FileReader(file))) {
            String line = br.readLine(); // Skip header: distance, robot_vel, angle, rpm, cost
            
            while ((line = br.readLine()) != null) {
                String[] values = line.split(",");
                double d = Double.parseDouble(values[0]);
                double v = Double.parseDouble(values[1]);
                double angle = Double.parseDouble(values[2]);
                double rpm = Double.parseDouble(values[3]);

                if (!distList.contains(d)) distList.add(d);
                if (!velList.contains(v)) velList.add(v);
                
                dataRows.add(new Double[]{d, v, angle, rpm});
            }

            // Apache requires sorted, unique arrays for the grid axes
            double[] xArr = distList.stream().sorted().mapToDouble(Double::doubleValue).toArray();
            double[] yArr = velList.stream().sorted().mapToDouble(Double::doubleValue).toArray();

            minDistance = xArr[0];
            maxDistance = xArr[xArr.length - 1];
            minVel = yArr[0];
            maxVel = yArr[yArr.length - 1];

            double[][] angleGrid = new double[xArr.length][yArr.length];
            double[][] rpmGrid = new double[xArr.length][yArr.length];

            // Map the flat list into the 2D grid
            for (Double[] row : dataRows) {
                int xIdx = binarySearch(xArr, row[0]);
                int yIdx = binarySearch(yArr, row[1]);
                angleGrid[xIdx][yIdx] = row[2];
                rpmGrid[xIdx][yIdx] = row[3];
            }

            BicubicInterpolator interpolator = new BicubicInterpolator();
            this.angleSpline = interpolator.interpolate(xArr, yArr, angleGrid);
            this.rpmSpline = interpolator.interpolate(xArr, yArr, rpmGrid);

        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Predicts the RPM and Angle for a given distance and robot velocity.
     * Inputs are clamped to the range of the dataset to prevent crashes.
     */
    public ShooterSetpoint getSetpoint(double distance, double velocity) {
        double d = Math.max(minDistance, Math.min(maxDistance, distance));
        double v = Math.max(minVel, Math.min(maxVel, velocity));

        return new ShooterSetpoint(
            angleSpline.value(d, v),
            rpmSpline.value(d, v)
        );
    }

    private int binarySearch(double[] arr, double target) {
        int idx = java.util.Arrays.binarySearch(arr, target);
        return idx < 0 ? -(idx + 1) : idx;
    }

    public record ShooterSetpoint(double angle, double rpm) {}
}