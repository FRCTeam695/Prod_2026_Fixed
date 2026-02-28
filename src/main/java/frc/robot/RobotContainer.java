
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.BisonLib.BaseProject.Controller.EnhancedCommandController;
import frc.BisonLib.BaseProject.Swerve.Modules.TalonFXModule;
import frc.BisonLib.BaseProject.Util.SOTMSetpointGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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



  private SendableChooser<Command> autoChooser = new SendableChooser<>();
  public SOTMSetpointGenerator shotCalculator;
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

    SmartDashboard.putData("Swerve Subsystem", swerve);
    shotCalculator = new SOTMSetpointGenerator(swerve::getSavedPose, swerve::getLatestChassisSpeed);


    configureBindings();
    configureDefaultCommands();

      
    autoChooser.addOption("oupost to depot", 
          swerve.resetGyroWithAllianceFlip(90)
          .andThen
          (
            parallel
            (
              pivot.goToPositionDegreesWithCondition(pivot.pivotExtendedPositionDegrees, pivot.withinTolerance),
              swerve.driveToPose(new Pose2d(0, 0, new Rotation2d()), 0.01) // find this pose (outpost intake pose)
            )
          )
          .andThen
          (
            new WaitCommand(0.3) // tune this number
          )
          .andThen
          (
            swerve.driveToPose(new Pose2d(0, 0, new Rotation2d()), 2) //find this pose (intermediate pose)
            .andThen
            (
              swerve.driveToPose(new Pose2d(0, 0, new Rotation2d()), 2) //find this pose (intermediate pose)
            )
          )
          .andThen
          (
            swerve.driveToPose(new Pose2d(0, 0, new Rotation2d()), 0.01) // find this pose (prepare depot intake pose)
          )
          .andThen
          (
            parallel
            (
              run
              (
                ()-> swerve.driveRobotRelative(new ChassisSpeeds(1.5, 0, 0), true),
                swerve
              ),
              intakeRollers.setDutyCycle(()-> Constants.Intake.INTAKE_SPEED)
            ).withTimeout(1.5)
          )
    );
    autoChooser.addOption("depot to outpost", new WaitCommand(0));
    autoChooser.addOption("depot", new WaitCommand(0));
    autoChooser.addOption("outpost", new WaitCommand(0));
    SmartDashboard.putData(autoChooser);

    DataLogManager.start();
  }

  public Runnable getOdometryUpdater(){
    return swerve::updateOdometryWithKinematics;
  }


  private void configureBindings() {
    driver.back().onTrue(swerve.backwardsResetGyro());

    /*
     * Intake while held
     */
    driver.leftTrigger(0.5).whileTrue(
          pivot.goToPositionDegreesWithCondition(pivot.pivotExtendedPositionDegrees, pivot.withinTolerance)
          .andThen(
          parallel(
            pivot.setDutyCycle(()-> -0.1),
            intakeRollers.setDutyCycle(()-> Constants.Intake.INTAKE_SPEED)
          )
          )
    );
    

    /*
     * Auto shoot while held
     */
    driver.rightTrigger().whileTrue(
      swerve.setHubTagsValid().andThen(
        parallel(
          tripleShooter.setVelocityTorqueCurrentMPS(()-> shotCalculator.getCachedSetpoint().rpm()),
          swerve.rotateTowardsVirtualHub(driver::getRequestedChassisSpeeds, ()-> shotCalculator.getCachedSetpoint().virtualTarget())
        ).until(tripleShooter.allShootersWithinTolerance.and(swerve.atRotationSetpoint))
      )
      .andThen(
          kicker.setVelocityMPS(()-> kicker.maxSpeedRPS * kicker.surfaceMetersPerMotorRotation).until(kicker.velAboveThreshold)
      )
      .andThen(
        parallel(
          kicker.setVelocityMPS(()-> kicker.maxSpeedRPS * kicker.surfaceMetersPerMotorRotation),
          tripleShooter.setVelocityTorqueCurrentMPS(()-> shotCalculator.getCachedSetpoint().rpm()),
          feeder.setVelocityMPS(()-> shotCalculator.getCachedSetpoint().feedSpeed()),
          pivot.slowRaise(),
          swerve.rotateTowardsVirtualHub(driver::getRequestedChassisSpeeds, ()-> shotCalculator.getCachedSetpoint().virtualTarget())
        )
      )
    );
    driver.rightTrigger().onFalse(swerve.setAllTagsValid());

    /*
     * stockpile while held
     */
    driver.rightBumper().whileTrue(
      (
        parallel(
          tripleShooter.setVelocityTorqueCurrentMPS(()-> tripleShooter.kMaxSpeedMPS * Constants.Shooter.STOCKPILE_SPEED_PERCENT),
          swerve.rotateTowardsVirtualHub(driver::getRequestedChassisSpeeds, ()-> shotCalculator.getClosestStockpileTarget())
        ).until(tripleShooter.allShootersWithinTolerance)
        .andThen(
            kicker.setVelocityMPS(()-> kicker.maxSpeedRPS * kicker.surfaceMetersPerMotorRotation).until(kicker.velAboveThreshold)
        )
        .andThen(
          parallel(
            swerve.rotateTowardsVirtualHub(driver::getRequestedChassisSpeeds, ()-> shotCalculator.getClosestStockpileTarget()),
            kicker.setVelocityMPS(()-> kicker.maxSpeedRPS * kicker.surfaceMetersPerMotorRotation),
            tripleShooter.setVelocityTorqueCurrentMPS(()-> tripleShooter.kMaxSpeedMPS * Constants.Shooter.STOCKPILE_SPEED_PERCENT),
            feeder.setVelocityMPS(()-> -2.0),
            pivot.slowRaise()
          )
        )
      ).alongWith(
        hood.setActuatorDeg(()-> 55.0)
      )
    );

    /*
     * home the pivot while held
     */
    driver.povUp().onTrue(
      pivot.homePivotToRetracted()
    );

    /*
     * home pivot but going down
     */
    driver.povDown().onTrue(
      pivot.homePivotToExtended()
    );

    /*
     * dump balls if they are stuck in the vertical column
     */
    driver.a().whileTrue(
      parallel(
        tripleShooter.setDutyCycle(()-> 0.4),
        kicker.setDutyCycle(()-> 1)
      )
    );

    /*
     * retract pivot
     */
    driver.b().onTrue(pivot.setPositionDegrees(()-> pivot.pivotRetractedPositionDegrees));

    /*
     * auto rotate and set speed limit so that we dont get beached on fuel/bump
     */
    driver.leftBumper().whileTrue(
      swerve.safeTraverseBump(driver::getRequestedChassisSpeeds)
    );

    /*
     * x lock wheels
     */
    driver.y().whileTrue(
      swerve.xLockWheels()
    );

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
    hood.setDefaultCommand(hood.setActuatorDeg(()-> shotCalculator.getCachedSetpoint().angle()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}