// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.BisonLib.BaseProject.Controller.EnhancedCommandController;
import frc.BisonLib.BaseProject.Swerve.Modules.TalonFXModule;
import frc.BisonLib.BaseProject.Util.SOTMSetpointGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
  private final Hood hood;


  public final InterpolatingDoubleTreeMap distanceToShotSpeedTable;



  private SendableChooser<Command> autoChooser = new SendableChooser<>();
  SOTMSetpointGenerator shooterInterpolationMap;
  public int[] reefTags = {1,2,3,4,5,6,7,8,9,10,11,17,18,19,20,21,22,24, 25, 26, 27, 28, 29, 30, 31, 32, 33};

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
                                new ShooterMiniConfig(InvertedValue.Clockwise_Positive, 9.5, 0.0, 2.7, 0.0, "left", 54), //2.7, 9
                                new ShooterMiniConfig(InvertedValue.CounterClockwise_Positive, 9.3, 0.0, 3.5, 0.0,"middle", 55),
                                new ShooterMiniConfig(InvertedValue.CounterClockwise_Positive, 10.2, 0.0, 2.0, 0.0,"right", 52)
                              );
    pivot = new IntakePivot();
    hood = new Hood();

    distanceToShotSpeedTable = new InterpolatingDoubleTreeMap();
    distanceToShotSpeedTable.put(0.0, 0.0);

    SmartDashboard.putData("Swerve Subsystem", swerve);
    shooterInterpolationMap = new SOTMSetpointGenerator("simulated_optimal_trajectories.csv", swerve::getSavedPose, swerve::getLatestChassisSpeed);


    configureBindings();
    configureDefaultCommands();

      
    SmartDashboard.putData(autoChooser);

    DataLogManager.start();
  }

  public Runnable getOdometryUpdater(){
    return swerve::updateOdometryWithKinematics;
  }


  private void configureBindings() {
    driver.back().onTrue(swerve.backwardsResetGyro());

    driver.leftTrigger(0.5).whileTrue(
          pivot.goToPositionDegreesWithCondition(pivot.pivotExtendedPositionDegrees, pivot.withinTolerance)
          .andThen(
          parallel(
            pivot.setDutyCycle(()-> -0.1),
            intakeRollers.setVelocityRPS(()-> 70)
          )
          )
    );
    

    //2.52, 4.03 is limelight pose for this shot
    // 0.97, 4.13
    // 2.14, 4.09
    driver.rightTrigger().whileTrue(
      deadline(
        //tripleShooter.setVelocityMPSWithCondition(()-> tripleShooter.kMaxSpeedMPS * 0.6, tripleShooter.allShootersWithinTolerance)
        tripleShooter.setVelocityTorqueCurrentMPS(()-> tripleShooter.kMaxSpeedMPS * 0.60).until(tripleShooter.allShootersWithinTolerance)
        //hood.setActuatorDeg(70.)
      )
      .andThen(
          kicker.setVelocityMPS(()-> kicker.maxSpeedRPS * kicker.surfaceMetersPerMotorRotation).until(kicker.velAboveThreshold)
      )
      .andThen(
        parallel(
          kicker.setVelocityMPS(()-> kicker.maxSpeedRPS * kicker.surfaceMetersPerMotorRotation),
          tripleShooter.setVelocityTorqueCurrentMPS(()-> tripleShooter.kMaxSpeedMPS * 0.60),
          feeder.setVelocityMPS(()-> -1.0),
          pivot.slowRaise()
          //hood.setActuatorDeg(70.)
        )
      )
    );

    driver.rightBumper().onTrue(
      pivot.homePivotToRetracted()

    );

    // driver.a().whileTrue(
    //   tripleShooter.setVelocityTorqueCurrentMPS(()-> tripleShooter.kMaxSpeedMPS * -driver.getLeftY())
    //   // kicker.setVelocityMPS(()-> kicker.maxSpeedRPS * kicker.surfaceMetersPerMotorRotation * -driver.getLeftY())
    // );
    driver.b().whileTrue(hood.setActuatorDeg(74));
    driver.a().whileTrue(hood.setActuatorDeg(66));

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
          ).withName("Swerve Drive Command")
      );
    feeder.setDefaultCommand(feeder.openLoopSet(()-> 0));
    intakeRollers.setDefaultCommand(intakeRollers.setDutyCycle(()-> 0));
    kicker.setDefaultCommand(kicker.setDutyCycle(()-> 0));
    tripleShooter.setDefaultCommand(tripleShooter.setVelocityTorqueCurrentMPS(()-> 0));
    pivot.setDefaultCommand(pivot.setDutyCycle(()-> 0));
    hood.setDefaultCommand(hood.setActuatorDeg(65.17));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}