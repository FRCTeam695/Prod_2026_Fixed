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


import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.subsystems.*;
import frc.robot.subsystems.IndividualShooter.ShooterMiniConfig;


public class RobotContainer {

  private final RebuiltSwerve swerve;
  private final Feeder feeder;
  private final IntakeRollers intakeRollers;
  private final Kicker kicker;
  private final TripleShooter tripleShooter;
  private final IntakePivot pivot;


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
    tripleShooter = new TripleShooter(
                                new ShooterMiniConfig(InvertedValue.Clockwise_Positive, 0.14362, 0.115, 0.11821, 0.012983, "left", 54),
                                new ShooterMiniConfig(InvertedValue.CounterClockwise_Positive, 0.060303, 0.1145, 0.12779, 0.012318,"middle", 55),
                                new ShooterMiniConfig(InvertedValue.CounterClockwise_Positive, 0.076057, 0.116, 0.14611, 0.013726,"right", 52)
                              );
    pivot = new IntakePivot();


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

    driver.leftTrigger(0.5).whileTrue(
        pivot.setPositionDegrees(pivot.pivotExtendedPositionDegrees).until(pivot.atSetpoint)
        .andThen(
          parallel(
            pivot.setDutyCycle(()-> -0.1),
            intakeRollers.setVelocityRPS(()-> 70)
          )
        )
    );
    driver.b().onTrue(pivot.setPositionDegrees(pivot.pivotRetractedPositionDegrees));

    driver.rightTrigger().whileTrue(
      tripleShooter.setVelocityMPS(()-> tripleShooter.kMaxSpeedMPS * 0.5).withTimeout(3)
      .andThen(
        parallel(
          kicker.setDutyCycle(()-> 1),
          tripleShooter.setVelocityMPS(()-> tripleShooter.kMaxSpeedMPS * 0.5),
          feeder.setVelocityMPS(()-> -2)
        )
      )
    );

    driver.rightBumper().onTrue(
      pivot.homePivot()
    );
  }



  public void configureDefaultCommands(){
    // swerve.setDefaultCommand
    //   (
        
    //     run
    //       (
    //         ()-> 
    //           swerve.teleopDefaultCommand(
    //             driver::getRequestedChassisSpeeds,
    //             true
    //           )
    //           ,
    //           swerve
    //       ).withName("Swerve Drive Command")
    //   );
    feeder.setDefaultCommand(feeder.openLoopSet(()-> 0));
    intakeRollers.setDefaultCommand(intakeRollers.setDutyCycle(()-> 0));
    kicker.setDefaultCommand(kicker.setDutyCycle(()-> 0));
    tripleShooter.setDefaultCommand(tripleShooter.setVelocityMPS(()-> 0));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}