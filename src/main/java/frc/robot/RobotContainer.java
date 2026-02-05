// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.BisonLib.BaseProject.Controller.EnhancedCommandController;
import frc.BisonLib.BaseProject.Swerve.SwerveBase;
import frc.BisonLib.BaseProject.Swerve.Modules.TalonFXModule;
import frc.BisonLib.BaseProject.Util.ShootOnTheMoveHelper;

import edu.wpi.first.wpilibj2.command.Command;



import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  SendableChooser<Command> autoChooser = new SendableChooser<>();
  ShootOnTheMoveHelper shooterInterpolationMap;

  private static final EnhancedCommandController driver = new EnhancedCommandController(0);
  public final SwerveBase swerve;
  private final String[] camNames = {"limelight-left", "limelight-right"};

  public int[] reefTags = {6,7,8,9,10,11,17,18,19,20,21,22};
  private final TalonFXModule[] modules = new TalonFXModule[] 
        {
          new TalonFXModule(Constants.Swerve.FRONT_RIGHT_DRIVE_ID, Constants.Swerve.FRONT_RIGHT_TURN_ID, Constants.Swerve.FRONT_RIGHT_ABS_ENCODER_OFFSET_ROTATIONS, Constants.Swerve.FRONT_RIGHT_CANCODER_ID, 0),
          new TalonFXModule(Constants.Swerve.FRONT_LEFT_DRIVE_ID, Constants.Swerve.FRONT_LEFT_TURN_ID, Constants.Swerve.FRONT_LEFT_ABS_ENCODER_OFFSET_ROTATIONS, Constants.Swerve.FRONT_LEFT_CANCODER_ID, 1),
          new TalonFXModule(Constants.Swerve.BACK_LEFT_DRIVE_ID, Constants.Swerve.BACK_LEFT_TURN_ID, Constants.Swerve.BACK_LEFT_ABS_ENCODER_OFFSET_ROTATIONS, Constants.Swerve.BACK_LEFT_CANCODER_ID, 2),
          new TalonFXModule(Constants.Swerve.BACK_RIGHT_DRIVE_ID, Constants.Swerve.BACK_RIGHT_TURN_ID, Constants.Swerve.BACK_RIGHT_ABS_ENCODER_OFFSET_ROTATIONS, Constants.Swerve.BACK_RIGHT_CANCODER_ID, 3)
        };
  

  public RobotContainer() {
    swerve = new SwerveBase(camNames, modules, reefTags);
    shooterInterpolationMap = new ShootOnTheMoveHelper("simulated_optimal_trajectories.csv", ()-> swerve.getSavedPose(), ()-> swerve.getLatestChassisSpeed());

    // Configure the trigger bindings
    configureBindings();

      
    SmartDashboard.putData(autoChooser);

    DataLogManager.start();
  }


  private void configureBindings() {
    driver.a().onTrue(getAutonomousCommand());
  }

  public Runnable getOdometryUpdater(){
    return swerve::updateOdometryWithKinematics;
  }


  // The command specified in here is run in autonomous
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}