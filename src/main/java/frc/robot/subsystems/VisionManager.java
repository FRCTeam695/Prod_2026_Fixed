// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class VisionManager extends SubsystemBase {
  /** Creates a new VisionManager. */

  public int numFuelSeen;
  public List<Translation2d> fuelLocations = new ArrayList<Translation2d>();

  public String[] camNames;
  public PhotonCamera[] cameras;
  public Supplier<Pose2d> robotPoseSupplier;

  public VisionManager(String[] camNames, Supplier<Pose2d> robotPoseSupplier) {
    this.camNames = camNames;
    this.robotPoseSupplier = robotPoseSupplier;
    cameras = new PhotonCamera[camNames.length];
    for (int i = 0; i < camNames.length; i++) {
      cameras[i] = new PhotonCamera(camNames[i]);
    }
  }

  private Optional<Translation2d> getTranslationToFuelFieldRelative(Optional<Double> pitch_Optional, Optional<Double> yaw_Optional){
    if (pitch_Optional.isEmpty() || yaw_Optional.isEmpty()){
      return Optional.empty();
    }

    Translation2d fuelTranslationRobotRelative = getTranslationToFuelRobotRelative(pitch_Optional,yaw_Optional).get();
    //Robot Relative Pose to Field Relative Pose
    Pose2d robotPose = robotPoseSupplier.get();
    Translation2d fuelTranslationFieldRelative = fuelTranslationRobotRelative.rotateBy(robotPose.getRotation());
    fuelTranslationFieldRelative = fuelTranslationFieldRelative.plus(robotPose.getTranslation());

    return Optional.of(fuelTranslationFieldRelative);
  }

  private Optional<Translation2d> getTranslationToFuelRobotRelative(Optional<Double> pitch_Optional, Optional<Double> yaw_Optional){
    if (pitch_Optional.isEmpty() || yaw_Optional.isEmpty()){
      return Optional.empty();
    }
    double pitch = pitch_Optional.get();
    double yaw = yaw_Optional.get();

    /*  TO ADD: FUEL CUTOFF CORRECTION */

    double distanceToFuel = (Constants.Vision.FUEL_HEIGHT - Constants.Vision.CAMERA_HEIGHT) /
                    Math.tan(Math.toRadians(Constants.Vision.MOUNT_PITCH + pitch)) /
                    Math.cos(Math.toRadians(Constants.Vision.MOUNT_YAW_DIST + yaw));
    
    //negative yaw because ccw+ is the convention for Rotation2d, but cw+ for PhotonVision
    Rotation2d angleToFuel = new Rotation2d(Math.toRadians(-yaw + Constants.Vision.MOUNT_YAW_ANGLE));
    Translation2d fuelTranslationRobotRelative = new Translation2d(distanceToFuel, angleToFuel);
    return Optional.of(fuelTranslationRobotRelative);
  }

  public Translation2d getClosestFuelFieldRelative(){
    double minDistance = fuelLocations.get(0).getNorm();
    Translation2d closestFuel = fuelLocations.get(0);
    for (Translation2d fuel: fuelLocations){
      double dist = fuel.getNorm();
      if ( minDistance > dist){
        closestFuel = fuel;
        minDistance = dist;
      }
    }
    return closestFuel;
  }

  @Override
  public void periodic() {
    fuelLocations.clear();
    for ( PhotonCamera camera: cameras) {
      PhotonPipelineResult results = camera.getLatestResult();
      if (results.hasTargets()) {
        List<PhotonTrackedTarget> targets = results.getTargets();
        for (int i = 0; i < targets.size(); i++) {
          PhotonTrackedTarget fuel = targets.get(i);
          Optional<Translation2d> location_Optional = getTranslationToFuelFieldRelative(Optional.of(fuel.getPitch()), Optional.of(fuel.getYaw()));
          if ( location_Optional.isPresent() ) {
            fuelLocations.add(location_Optional.get());
          }
        }
      }
      numFuelSeen = fuelLocations.size();
    }
  }
}