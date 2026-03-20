
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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

    // SmartDashboard.putData("Swerve Subsystem", swerve);
    shotCalculator = new SOTMSetpointGenerator(swerve::getSavedPose, swerve::getLatestChassisSpeed);


    configureBindings();
    configureDefaultCommands();

    autoChooser.addOption("Citrus Sweep Right",
        (swerve.resetGyroWithAllianceFlip(-90)
          .andThen(
            swerve.setAllianceHubTagsValid()
          )
          .andThen(
            deadline(
              swerve.driveToPoseWithSpeedLimit(() -> swerve.flipPoseToRight(swerve.optionalFlipPoseSeededRed(new Pose2d(10.84, 2.3, Rotation2d.fromDegrees(90)))), 0.08, 2.5, 2.0)   //find this pose, right next to bump
              .andThen(swerve.driveToPose(() -> swerve.flipPoseToRight(swerve.optionalFlipPoseSeededRed(new Pose2d(9.87, 1.9, Rotation2d.fromDegrees(167.07)))), 0.1, 1.5)) //find this pose, get ready for pacman
              .andThen(
                  swerve.driveToPoseWithSpeedLimit(()-> swerve.flipPoseToRight(swerve.optionalFlipPoseSeededRed(new Pose2d(9.3, 1.9, Rotation2d.fromDegrees(167.07)))), 0.15, 2.0, 0.7) //inital centerline move
              ),
              pivot.goToPositionDegreesWithCondition(pivot.pivotExtendedPositionDegrees, pivot.withinTolerance).andThen(pivot.setDutyCycle(()-> -0.05)),             
              parallel(
                    tripleShooter.setDutyCycle(()-> -0.4),
                    kicker.setDutyCycle(()-> -1)
                  ).withTimeout(3).andThen(parallel(tripleShooter.setDutyCycle(()-> 0.0), kicker.setDutyCycle(()-> 0)))
            )
          )
          .andThen(
            deadline(
              swerve.driveToPoseWithSpeedLimit(() -> swerve.flipPoseToRight(swerve.optionalFlipPoseSeededRed(new Pose2d(8.15, 3.5, Rotation2d.fromDegrees(77.07)))), 0.08, 2.0, 2.0),
              pivot.setDutyCycle(()-> -0.05)
            )
          )
          .andThen(
            deadline(
              swerve.driveToPoseWithSpeedLimit(() -> swerve.flipPoseToRight(swerve.optionalFlipPoseSeededRed(new Pose2d(9.4, 3.64, Rotation2d.fromDegrees(0)))), 0.2, 2.0, 1.5),
              pivot.setDutyCycle(()-> -0.05)
          ))
          .andThen(
            deadline(
              swerve.driveToPoseWithSpeedLimit(() -> swerve.flipPoseToRight(swerve.optionalFlipPoseSeededRed(new Pose2d(10.68, 2.8, Rotation2d.fromDegrees(-60)))), 0.15, 2.0, 0.5),
              pivot.setDutyCycle(()-> -0.05)
          ))
          .andThen(
              deadline(
                swerve.driveToPoseWhileTurningToHub(() -> swerve.flipPoseToRight(swerve.optionalFlipPoseSeededRed(new Pose2d(13.9, 2.7, Rotation2d.fromDegrees(133.53)))), 0.4, 1.0),
                new WaitCommand(0.5).andThen(tripleShooter.setVelocityTorqueCurrentMPS(()-> shotCalculator.getCachedSetpoint().shotVelocityMPS()))
              )
          )
          .andThen(
            autoShoot().withTimeout(3.8)
          )
          .andThen(
            deadline(
              swerve.driveToPoseWithSpeedLimit(() -> swerve.flipPoseToRight(swerve.optionalFlipPoseSeededRed(new Pose2d(10.84, 2.3, Rotation2d.fromDegrees(90)))), 0.08, 2.5, 2.0)   //find this pose, right next to bump
              .andThen(swerve.driveToPose(() -> swerve.flipPoseToRight(swerve.optionalFlipPoseSeededRed(new Pose2d(9.87, 1.9, Rotation2d.fromDegrees(167.07)))), 0.1, 1.5)) //find this pose, get ready for pacman
              .andThen(
                  swerve.driveToPoseWithSpeedLimit(()-> swerve.flipPoseToRight(swerve.optionalFlipPoseSeededRed(new Pose2d(9.3, 1.9, Rotation2d.fromDegrees(167.07)))), 0.15, 2.0, 2.0)
              ),
              pivot.goToPositionDegreesWithCondition(pivot.pivotExtendedPositionDegrees, pivot.withinTolerance).andThen(pivot.setDutyCycle(()-> -0.05)),             
              parallel(
                    tripleShooter.setDutyCycle(()-> -0.4),
                    kicker.setDutyCycle(()-> -1)
                  ).withTimeout(3).andThen(parallel(tripleShooter.setDutyCycle(()-> 0.0), kicker.setDutyCycle(()-> 0)))
            )
          )
          .andThen(
            deadline(
              swerve.driveToPoseWithSpeedLimit(() -> swerve.flipPoseToRight(swerve.optionalFlipPoseSeededRed(new Pose2d(8.15, 3.5, Rotation2d.fromDegrees(77.07)))), 0.08, 2.0, 2.0),
              pivot.setDutyCycle(()-> -0.05)
            )
          )
          .andThen(
            deadline(
              swerve.driveToPose(() -> swerve.flipPoseToRight(swerve.optionalFlipPoseSeededRed(new Pose2d(9.4, 3.64, Rotation2d.fromDegrees(0)))), 0.2, 1.5),
              pivot.setDutyCycle(()-> -0.05)
          ))
          .andThen(
            deadline(
              swerve.driveToPose(() -> swerve.flipPoseToRight(swerve.optionalFlipPoseSeededRed(new Pose2d(10.68, 2.8, Rotation2d.fromDegrees(-60)))), 0.15, 0.5),
              pivot.setDutyCycle(()-> -0.05)
          ))
          .andThen(
              deadline(
                swerve.driveToPoseWhileTurningToHub(() -> swerve.flipPoseToRight(swerve.optionalFlipPoseSeededRed(new Pose2d(13.9, 2.7, Rotation2d.fromDegrees(133.53)))), 0.4, 1.0),
                new WaitCommand(0.5).andThen(tripleShooter.setVelocityTorqueCurrentMPS(()-> shotCalculator.getCachedSetpoint().shotVelocityMPS()))
              )
          )
          .andThen(
            autoShoot().withTimeout(3.8)
          )
          ).alongWith(new WaitCommand(0.5).andThen(intakeRollers.setVelocityRPS(()-> Constants.Intake.INTAKE_SPEED * intakeRollers.kMaxVelocity)))
    );

    autoChooser.addOption("Citrus Sweep Left", 
          (swerve.resetGyroWithAllianceFlip(90)
          .andThen(
            swerve.setAllianceHubTagsValid()
          )
          .andThen(
            deadline(
              swerve.driveToPoseWithSpeedLimit(() -> swerve.optionalFlipPoseSeededRed(new Pose2d(10.84, 2.3, Rotation2d.fromDegrees(90))), 0.08, 2.5, 2.0)   //find this pose, right next to bump
              .andThen(swerve.driveToPose(() -> swerve.optionalFlipPoseSeededRed(new Pose2d(9.87, 1.9, Rotation2d.fromDegrees(167.07))), 0.1, 1.5)) //find this pose, get ready for pacman
              .andThen(
                  swerve.driveToPoseWithSpeedLimit(()-> swerve.optionalFlipPoseSeededRed(new Pose2d(9.3, 1.9, Rotation2d.fromDegrees(167.07))), 0.15, 2.0, 0.7) //inital centerline move
              ),
              pivot.goToPositionDegreesWithCondition(pivot.pivotExtendedPositionDegrees, pivot.withinTolerance).andThen(pivot.setDutyCycle(()-> -0.05)),             
              parallel(
                    tripleShooter.setDutyCycle(()-> -0.4),
                    kicker.setDutyCycle(()-> -1)
                  ).withTimeout(3).andThen(parallel(tripleShooter.setDutyCycle(()-> 0.0), kicker.setDutyCycle(()-> 0)))
            )
          )
          .andThen(
            deadline(
              swerve.driveToPoseWithSpeedLimit(() -> swerve.optionalFlipPoseSeededRed(new Pose2d(8.15, 3.5, Rotation2d.fromDegrees(77.07))), 0.08, 2.0, 2.0),
              pivot.setDutyCycle(()-> -0.05)
            )
          )
          .andThen(
            deadline(
              swerve.driveToPoseWithSpeedLimit(() -> swerve.optionalFlipPoseSeededRed(new Pose2d(9.4, 3.64, Rotation2d.fromDegrees(0))), 0.2, 2.0, 1.5),
              pivot.setDutyCycle(()-> -0.05)
          ))
          .andThen(
            deadline(
              swerve.driveToPoseWithSpeedLimit(() -> swerve.optionalFlipPoseSeededRed(new Pose2d(10.68, 2.8, Rotation2d.fromDegrees(-60))), 0.15, 2.0, 0.5),
              pivot.setDutyCycle(()-> -0.05)
          ))
          .andThen(
              deadline(
                swerve.driveToPoseWhileTurningToHub(() -> swerve.optionalFlipPoseSeededRed(new Pose2d(13.9, 2.7, Rotation2d.fromDegrees(133.53))), 0.4, 2.0),
                new WaitCommand(0.5).andThen(tripleShooter.setVelocityTorqueCurrentMPS(()-> shotCalculator.getCachedSetpoint().shotVelocityMPS()))
              )
          )
          .andThen(
            autoShoot().withTimeout(3.8)
          )
          .andThen(
            deadline(
              swerve.driveToPose(()-> swerve.optionalFlipPoseSeededRed(new Pose2d(15.1, 0.54, Rotation2d.fromDegrees(113.3))), 0.1, 1.0), //get ready to intake from depot
              pivot.setDutyCycle(()-> -0.05),
              parallel(
                    tripleShooter.setDutyCycle(()-> -0.4),
                    kicker.setDutyCycle(()-> -1)
                  ).withTimeout(0.8).andThen(parallel(tripleShooter.setDutyCycle(()-> 0.0), kicker.setDutyCycle(()-> 0)))
            )
          )
          .andThen(
            deadline(
              //15.936, 0.950
              swerve.driveToPose(()-> swerve.optionalFlipPoseSeededRed(new Pose2d(15.936, 0.845, Rotation2d.fromDegrees(71.49))), 0.1, 0.5), //traverse through depot
              pivot.setDutyCycle(()-> -0.05)
            )
          )
          .andThen(
            deadline(
              swerve.driveToPoseWithSpeedLimit(()-> swerve.optionalFlipPoseSeededRed(new Pose2d(15.936, 2.518, Rotation2d.fromDegrees(71.49))), 0.1, 2, 0.5), //traverse through depot
              pivot.setDutyCycle(()-> -0.05)
            )
          )
          .andThen(
            deadline(
              swerve.driveToPose(
                ()-> swerve.optionalFlipPoseSeededBlue(new Pose2d(2.13 - Units.inchesToMeters(0),Constants.FieldConstants.FIELD_WIDTH  - 2.36 - 0.15, Rotation2d.fromDegrees(-34.65))), 
                0.40,
                2.0),
              tripleShooter.setVelocityTorqueCurrentMPS(()-> shotCalculator.getCachedSetpoint().shotVelocityMPS() * 0.8)
            )
          )
          .andThen(autoShoot())
          ).alongWith(new WaitCommand(0.5).andThen(intakeRollers.setVelocityRPS(()-> Constants.Intake.INTAKE_SPEED * intakeRollers.kMaxVelocity)))
    );

    autoChooser.addOption("outpost", 
          swerve.resetGyroWithAllianceFlip(-90)
          .andThen
          (
            deadline
            (
              swerve.driveToPose(()-> swerve.getOutpostPose(), 0.01, 0.0),
              pivot.goToPositionDegreesWithCondition(pivot.pivotExtendedPositionDegrees, pivot.withinTolerance)
            ).withTimeout(3) // find this pose (outpost intake pose)
          )
          .andThen
          (
            runOnce(() -> {swerve.stopModules();}).andThen(new WaitCommand(3)) // tune this number
          ).
          andThen(
            swerve.driveToPose( 
              ()-> swerve.optionalFlipPoseSeededBlue(new Pose2d(1.5, 0.64, Rotation2d.fromDegrees(180))), 
              0.5, 
              0.0)
          ).andThen
          (
            deadline(
              swerve.driveToPose(
                ()-> swerve.optionalFlipPoseSeededBlue(new Pose2d(2.18,2.36, Rotation2d.fromDegrees(34.65))),
                 0.01,
                 0.0),
              tripleShooter.setVelocityTorqueCurrentMPS(()-> shotCalculator.getCachedSetpoint().shotVelocityMPS())
            )
          ).andThen(
            autoShoot()
          )
  
    );

    autoChooser.addOption("depot", 
          swerve.resetGyroWithAllianceFlip(90)
          .andThen
          (
            deadline
            (
              swerve.driveToPose(
                ()-> swerve.optionalFlipPoseSeededBlue(new Pose2d(1.019,6.3 - Units.inchesToMeters(6), Rotation2d.fromDegrees(180))), 
                0.01,
                0.0),
              parallel(
                pivot.goToPositionDegreesWithCondition(pivot.pivotExtendedPositionDegrees, pivot.withinTolerance),
                new WaitCommand(0.25).andThen(
                  parallel(
                    tripleShooter.setDutyCycle(()-> -0.4),
                    kicker.setDutyCycle(()-> -1)
                  ).withTimeout(1)
                )
              )
            ) 
          )
          .andThen(
            deadline(
              run
              (
                ()-> swerve.driveRobotRelative(new ChassisSpeeds(0.75, 0, 0)),
                swerve
              ).withTimeout(1.0)
              .andThen(
                run
              (
                ()-> swerve.driveRobotRelative(new ChassisSpeeds(-0.75, 0, 0)),
                swerve
              ).withTimeout(0.75)
              ).andThen(
                swerve.driveToPose(
                  ()-> swerve.optionalFlipPoseSeededBlue(new Pose2d(0.844,5.3 + Units.inchesToMeters(6), Rotation2d.fromDegrees(180))), 
                  0.01,
                  0.0)
              ).andThen(
                 run
              (
                ()-> swerve.driveRobotRelative(new ChassisSpeeds(0.75, 0, 0)),
                swerve
              ).withTimeout(1.0))
              .andThen(
                 run
              (
                ()-> swerve.driveRobotRelative(new ChassisSpeeds(-0.75, 0, 0)),
                swerve
              ).withTimeout(1.0))
              ,
              parallel(
                pivot.setDutyCycle(()-> -0.05),
                intakeRollers.setVelocityRPS(()-> Constants.Intake.INTAKE_SPEED * intakeRollers.kMaxVelocity)
              )
          )
          )
          .andThen
          (
            deadline(
              swerve.driveToPose(
                ()-> swerve.optionalFlipPoseSeededBlue(new Pose2d(2.13 - Units.inchesToMeters(0),Constants.FieldConstants.FIELD_WIDTH  - 2.36 - 0.15, Rotation2d.fromDegrees(-34.65))), 
                0.01,
                0.0),
              tripleShooter.setVelocityTorqueCurrentMPS(()-> shotCalculator.getCachedSetpoint().shotVelocityMPS())
            )
          ).andThen(
            autoShoot()
          ));


    SmartDashboard.putData(autoChooser);

  }

  public Runnable getOdometryUpdater(){
    return swerve::updateOdometryWithKinematics;
  }


  @SuppressWarnings("static-access")
  private void configureBindings() {
    driver.povLeft().and(()-> DriverStation.isDisabled()).onTrue(
       swerve.resetGyroWithAllianceFlip(90)
    );

    driver.povRight().and(()-> DriverStation.isDisabled()).onTrue(
       swerve.resetGyroWithAllianceFlip(-90)
    );

    driver.back().onTrue(swerve.backwardsResetGyro());

    driver.leftBumper().toggleOnTrue(
        pivot.goToPositionDegreesWithCondition(pivot.pivotExtendedPositionDegrees, pivot.withinTolerance)
          .andThen(
            parallel(
              pivot.setDutyCycle(()-> -0.05),
              intakeRollers.setVelocityRPS(()-> Constants.Intake.INTAKE_SPEED * intakeRollers.kMaxVelocity)
            )
          ).alongWith(swerve.pacmanDrive(driver::getRequestedChassisSpeeds, driver::getRightStickHeading))
    );

    driver.leftTrigger().whileTrue(
      pivot.goToPositionDegreesWithCondition(pivot.pivotExtendedPositionDegrees, pivot.withinTolerance)
          .andThen(
            parallel(
              pivot.setDutyCycle(()-> -0.05),
              intakeRollers.setVelocityRPS(()-> Constants.Intake.INTAKE_SPEED * intakeRollers.kMaxVelocity)
            )
          ).alongWith(
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
    ));

    /*
     * Auto shoot while held
     */
    driver.rightTrigger().whileTrue(
      autoShoot()
    );

    driver.rightTrigger().onFalse(
      (runOnce(() -> swerve.setValidTagIDs("allTags"))
      .andThen(
        parallel(
          tripleShooter.setVelocityMPSWithCondition(()-> 0, tripleShooter.allShootersWithinTolerance),
          kicker.setVelocityMPS(()-> 0).until(kicker.isStopped)
        )
        .andThen(
          parallel(
            tripleShooter.setDutyCycle(()-> -0.4),
            kicker.setDutyCycle(()-> -1)
          ).withTimeout(0.5)
        )
      )).alongWith(pivot.setPositionDegrees(()-> pivot.pivotExtendedPositionDegrees))
    );

    /*
     * stockpile while held
     */
    driver.rightBumper().whileTrue(
      (
        parallel(
          tripleShooter.setVelocityTorqueCurrentMPS(()-> shotCalculator.getStockpileSetpoint().shotVelocityMPS()),
          swerve.rotateTowardsVirtualHub(driver::getRequestedChassisSpeeds, ()-> shotCalculator.getClosestStockpileTarget())
        ).until(tripleShooter.allShootersWithinTolerance)
        .andThen(
            kicker.setVelocityMPS(()-> kicker.maxSpeedRPS * kicker.surfaceMetersPerMotorRotation).until(kicker.velAboveThreshold)
        )
        .andThen(
          parallel(
            swerve.rotateTowardsVirtualHub(driver::getRequestedChassisSpeeds, ()-> shotCalculator.getClosestStockpileTarget()),
            kicker.setVelocityMPS(()-> kicker.maxSpeedRPS * kicker.surfaceMetersPerMotorRotation),
            tripleShooter.setVelocityTorqueCurrentMPS(()-> shotCalculator.getStockpileSetpoint().shotVelocityMPS()),
            feeder.setVelocityMPS(()-> -feeder.metersPerRotationOfMotor * 100),
            pivot.slowRaise()
          )
        )
      ).alongWith(
        hood.setActuatorDeg(()-> shotCalculator.getStockpileSetpoint().angle())
      )
    );

    driver.rightBumper().onFalse(
      (parallel(
        tripleShooter.setVelocityMPSWithCondition(()-> 0, tripleShooter.allShootersWithinTolerance),
        kicker.setVelocityMPS(()-> 0).until(kicker.isStopped)
      )
      .andThen(
        parallel(
          tripleShooter.setDutyCycle(()-> -0.4),
          kicker.setDutyCycle(()-> -1)
        ).withTimeout(1.0)
      )).alongWith(pivot.setPositionDegrees(()-> pivot.pivotExtendedPositionDegrees))
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
    
     */
    driver.a().whileTrue(
      parallel(
        tripleShooter.setDutyCycle(()-> -0.4),
        kicker.setDutyCycle(()-> -1)
      )
    );

    /*
     * retract pivot
     */
    driver.b().onTrue(
      pivot.setPositionDegrees(()-> pivot.pivotRetractedPositionDegrees)
    );

    driver.povLeft().onTrue(
      //pivot.slowRaise()
      hood.setActuatorDeg(() -> 72)
    );

    driver.povRight().onTrue(
      hood.setActuatorDeg(()-> 52)
    );

    driver.y().whileTrue(
        tripleShooter.setVelocityTorqueCurrentMPS(
            ()-> shotCalculator.getCachedSetpoint().shotVelocityMPS()
        )
    );

    driver.x().onTrue(
      feeder.runFeederOneRotation().andThen(
        intakeRollers.setVelocityRPS(()-> -Constants.Intake.INTAKE_SPEED * intakeRollers.kMaxVelocity)
      ).withTimeout(2)
    );

    // driver.x().whileTrue(
    //     swerve.setHubTagsValid()
    //     .andThen(
    //       parallel(
    //         tripleShooter.setVelocityTorqueCurrentMPS(()-> 2 * Math.PI * Units.inchesToMeters(2) * 100 * 0.7),
    //         swerve.rotateTowardsVirtualHub(driver::getRequestedChassisSpeeds, ()-> shotCalculator.getCachedSetpoint().virtualTarget())
    //     ).until(tripleShooter.allShootersWithinTolerance.and(swerve.atRotationSetpoint))
    //     )
    //     .andThen(
    //       kicker.setVelocityMPS(()-> kicker.maxSpeedRPS * kicker.surfaceMetersPerMotorRotation).until(kicker.velAboveThreshold)
    //     )
    //     .andThen(
    //       parallel(
    //         kicker.setVelocityMPS(()-> kicker.maxSpeedRPS * kicker.surfaceMetersPerMotorRotation),
    //         tripleShooter.setVelocityTorqueCurrentMPS(()-> 2 * Math.PI * Units.inchesToMeters(2) * 100 * 0.7),
    //         feeder.setVelocityMPS(()-> shotCalculator.getCachedSetpoint().feedSpeed()),
    //         pivot.slowRaise(),
    //         swerve.rotateTowardsVirtualHub(driver::getRequestedChassisSpeeds, ()-> shotCalculator.getCachedSetpoint().virtualTarget())
    //       )
    //     )
    // );

  }

   public Command autoShoot(){
      return (
        swerve.setAllianceHubTagsValid()
        .andThen(
          parallel(
            tripleShooter.setVelocityTorqueCurrentMPS(()-> shotCalculator.getCachedSetpoint().shotVelocityMPS()),
            swerve.rotateTowardsVirtualHub(driver::getRequestedChassisSpeeds, ()-> shotCalculator.getCachedSetpoint().virtualTarget())
        ).until(tripleShooter.allShootersWithinTolerance.and(swerve.atRotationSetpoint))
        )
        .andThen(
          kicker.setVelocityMPS(()-> kicker.maxSpeedRPS * kicker.surfaceMetersPerMotorRotation).until(kicker.velAboveThreshold)
        )
        .andThen(
          parallel(
            kicker.setVelocityMPS(()-> kicker.maxSpeedRPS * kicker.surfaceMetersPerMotorRotation),
            tripleShooter.setVelocityTorqueCurrentMPS(()-> shotCalculator.getCachedSetpoint().shotVelocityMPS()),
            feeder.setVelocityMPS(()-> shotCalculator.getCachedSetpoint().feedSpeed()),
            pivot.slowRaise(),
            swerve.rotateTowardsVirtualHub(driver::getRequestedChassisSpeeds, ()-> shotCalculator.getCachedSetpoint().virtualTarget())
          )
        )
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
    tripleShooter.setDefaultCommand(tripleShooter.setDutyCycle(()-> 0));
    pivot.setDefaultCommand(pivot.setDutyCycle(()-> 0));
    hood.setDefaultCommand(
      hood.setActuatorDeg(
        () -> 
        shotCalculator.getCachedSetpoint().angle()
      )
    );
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}