// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.BisonLib.BaseProject.Controller.EnhancedCommandController;
import frc.BisonLib.BaseProject.Swerve.Modules.TalonFXModule;
import frc.BisonLib.BaseProject.Util.SOTMSetpointGenerator;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import frc.robot.subsystems.*;


public class RobotContainer {

  private final RebuiltSwerve swerve;
  private final Feeder feeder;
  private final IntakeRollers intakeRollers;
  private final Kicker kicker;


  SendableChooser<Command> autoChooser = new SendableChooser<>();
  SOTMSetpointGenerator shooterInterpolationMap;
  public int[] reefTags = {6,7,8,9,10,11,17,18,19,20,21,22};

  private final TalonFXModule[] modules = new TalonFXModule[]
  {
    new TalonFXModule(Constants.Swerve.FRONT_RIGHT_DRIVE_ID, Constants.Swerve.FRONT_RIGHT_TURN_ID, Constants.Swerve.FRONT_RIGHT_ABS_ENCODER_OFFSET_ROTATIONS, Constants.Swerve.FRONT_RIGHT_CANCODER_ID, 0),
    new TalonFXModule(Constants.Swerve.FRONT_LEFT_DRIVE_ID, Constants.Swerve.FRONT_LEFT_TURN_ID, Constants.Swerve.FRONT_LEFT_ABS_ENCODER_OFFSET_ROTATIONS, Constants.Swerve.FRONT_LEFT_CANCODER_ID, 1),
    new TalonFXModule(Constants.Swerve.BACK_LEFT_DRIVE_ID, Constants.Swerve.BACK_LEFT_TURN_ID, Constants.Swerve.BACK_LEFT_ABS_ENCODER_OFFSET_ROTATIONS, Constants.Swerve.BACK_LEFT_CANCODER_ID, 2),
    new TalonFXModule(Constants.Swerve.BACK_RIGHT_DRIVE_ID, Constants.Swerve.BACK_RIGHT_TURN_ID, Constants.Swerve.BACK_RIGHT_ABS_ENCODER_OFFSET_ROTATIONS, Constants.Swerve.BACK_RIGHT_CANCODER_ID, 3),
  };
  private final String[] camNames = {"limelight-shooter"};
  private static final EnhancedCommandController driver = new EnhancedCommandController(0);



  public RobotContainer() {
    swerve = new RebuiltSwerve(camNames, modules, reefTags);
    feeder = new Feeder();
    intakeRollers = new IntakeRollers();
    kicker = new Kicker();


    SmartDashboard.putData("Swerve Subsystem", swerve);
    shooterInterpolationMap = new SOTMSetpointGenerator("simulated_optimal_trajectories.csv", swerve::getSavedPose, swerve::getLatestChassisSpeed);


    configureBindings();
    configureDefaultCommands();
    configureDefaultCommands();

      
    SmartDashboard.putData(autoChooser);

    DataLogManager.start();
  }

  public Runnable getOdometryUpdater(){
    return swerve::updateOdometryWithKinematics;
  }


  private void configureBindings() {
    driver.back().onTrue(swerve.resetGyro());

    
  }

  public void configureDefaultCommands(){
    swerve.setDefaultCommand
      (
        
        run
          (
            ()-> 
              swerve.teleopDefaultCommand(
                driver::getRequestedChassisSpeeds,
                true
              )
              ,
              swerve
          ).withName("Swerve Drive Command"))
      ;

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}