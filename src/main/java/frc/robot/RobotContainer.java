
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
import edu.wpi.first.math.geometry.Translation2d;
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
  private final LED led;



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
    led = new LED();

    // SmartDashboard.putData("Swerve Subsystem", swerve);
    shotCalculator = new SOTMSetpointGenerator(swerve::getSavedPose, swerve::getLatestChassisSpeed);


    configureBindings();
    configureDefaultCommands();

    autoChooser.addOption("Mirror", 
      (
            swerve.setAllianceHubTagsValid()
          .andThen(
            deadline(
              swerve.driveToPoseWithSpeedLimit(() -> (swerve.optionalFlipPoseSeededRed(new Pose2d(10.84, 2.3, Rotation2d.fromDegrees(90)))), 0.08, 3, 2.5)   //find this pose, right next to bump
              .andThen(swerve.driveToPose(() -> (swerve.optionalFlipPoseSeededRed(new Pose2d(9.87, 1.9, Rotation2d.fromDegrees(167.07)))), 0.2, 3.0)) //find this pose, get ready for pacman
              .andThen(
                  swerve.driveToPoseWithSpeedLimit(()-> (swerve.optionalFlipPoseSeededRed(new Pose2d(9.3, 1.9, Rotation2d.fromDegrees(167.07)))), 0.15, 3.0, 3.0) //inital centerline move
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
              swerve.driveToPoseWithSpeedLimit(() -> (swerve.optionalFlipPoseSeededRed(new Pose2d(8.15, 3.5, Rotation2d.fromDegrees(77.07)))), 0.08, 2.5, 2.5),
              pivot.setDutyCycle(()-> -0.05)
            )
          )
          .andThen(
            deadline(
              swerve.driveToPoseWithSpeedLimit(() -> (swerve.optionalFlipPoseSeededRed(new Pose2d(9.4, 3.64, Rotation2d.fromDegrees(0)))), 0.2, 2.5, 2.5),
              pivot.setDutyCycle(()-> -0.05)
          ))
          .andThen(
            deadline(
              swerve.driveToPoseWithSpeedLimit(() -> (swerve.optionalFlipPoseSeededRed(new Pose2d(10.68, 2.7, Rotation2d.fromDegrees(-60)))), 0.15, 2.0, 0.5),
              pivot.setDutyCycle(()-> -0.05)
          ))
          .andThen(
              deadline(
                swerve.driveToPoseWhileTurningToHub(() -> (swerve.optionalFlipPoseSeededRed(new Pose2d(13.9, 2.7, Rotation2d.fromDegrees(133.53)))), 0.4, 1.0),
                new WaitCommand(0.5).andThen(tripleShooter.setVelocityTorqueCurrentMPS(()-> shotCalculator.getCachedSetpoint().shotVelocityMPS()))
              )
          )
          .andThen(
            autoShoot().withTimeout(3.8)
          )
          .andThen(
            deadline(
              swerve.driveToPoseWithSpeedLimit(() -> (swerve.optionalFlipPoseSeededRed(new Pose2d(10.84, 2.3, Rotation2d.fromDegrees(90)))), 0.08, 3, 2.5)   //find this pose, right next to bump
              .andThen(swerve.driveToPose(() -> (swerve.optionalFlipPoseSeededRed(new Pose2d(9.87, 1.9, Rotation2d.fromDegrees(167.07)))), 0.1, 3.0)) //find this pose, get ready for pacman
              .andThen(
                  swerve.driveToPoseWithSpeedLimit(()-> (swerve.optionalFlipPoseSeededRed(new Pose2d(9.3, 1.9, Rotation2d.fromDegrees(167.07)))), 0.15, 3.0, 3.0)
              ),
              pivot.goToPositionDegreesWithCondition(pivot.pivotExtendedPositionDegrees, pivot.withinTolerance).andThen(pivot.setDutyCycle(()-> -0.05)),             
              parallel(
                    tripleShooter.setDutyCycle(()-> -0.4),
                    kicker.setDutyCycle(()-> -1)
                  ).withTimeout(3).andThen(parallel(tripleShooter.setDutyCycle(()-> 0.0), kicker.setDutyCycle(()-> 0))),
              feeder.openLoopSet(()-> 0)
            )
          )
          .andThen(
            deadline(
              swerve.driveToPoseWithSpeedLimit(() -> (swerve.optionalFlipPoseSeededRed(new Pose2d(8.15, 3.5, Rotation2d.fromDegrees(77.07)))), 0.08, 2.5, 2.5),
              pivot.setDutyCycle(()-> -0.05)
            )
          )
          .andThen(
            deadline(
              swerve.driveToPose(() -> (swerve.optionalFlipPoseSeededRed(new Pose2d(9.4, 3.64, Rotation2d.fromDegrees(0)))), 0.2, 2.5),
              pivot.setDutyCycle(()-> -0.05)
          ))
          .andThen(
            deadline(
              swerve.driveToPose(() -> (swerve.optionalFlipPoseSeededRed(new Pose2d(10.68, 2.7, Rotation2d.fromDegrees(-60)))), 0.15, 0.5),
              pivot.setDutyCycle(()-> -0.05)
          ))
          .andThen(
              deadline(
                swerve.driveToPoseWhileTurningToHub(() -> (swerve.optionalFlipPoseSeededRed(new Pose2d(13.9, 2.7, Rotation2d.fromDegrees(133.53)))), 0.4, 1.0),
                new WaitCommand(0.5).andThen(tripleShooter.setVelocityTorqueCurrentMPS(()-> shotCalculator.getCachedSetpoint().shotVelocityMPS()))
              )
          )
          .andThen(
            autoShoot().withTimeout(3.8)
          )
          ).alongWith(new WaitCommand(0.5).andThen(intakeRollers.setVelocityRPS(()-> Constants.Intake.INTAKE_SPEED * intakeRollers.kMaxVelocity)))
    );

    autoChooser.addOption("Citrus Sweep Right",
        (
            swerve.setAllianceHubTagsValid()
          .andThen(
            deadline(
              swerve.driveToPoseWithSpeedLimit(() -> swerve.flipPoseToRight(swerve.optionalFlipPoseSeededRed(new Pose2d(10.84, 2.3, Rotation2d.fromDegrees(90)))), 0.08, 3, 2.5)   //find this pose, right next to bump
              .andThen(swerve.driveToPose(() -> swerve.flipPoseToRight(swerve.optionalFlipPoseSeededRed(new Pose2d(9.87, 1.9, Rotation2d.fromDegrees(167.07)))), 0.2, 3.0)) //find this pose, get ready for pacman
              .andThen(
                  swerve.driveToPoseWithSpeedLimit(()-> swerve.flipPoseToRight(swerve.optionalFlipPoseSeededRed(new Pose2d(9.3, 1.9, Rotation2d.fromDegrees(167.07)))), 0.15, 3.0, 3.0) //inital centerline move
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
              swerve.driveToPoseWithSpeedLimit(() -> swerve.flipPoseToRight(swerve.optionalFlipPoseSeededRed(new Pose2d(8.15, 3.5, Rotation2d.fromDegrees(77.07)))), 0.08, 2.5, 2.5),
              pivot.setDutyCycle(()-> -0.05)
            )
          )
          .andThen(
            deadline(
              swerve.driveToPoseWithSpeedLimit(() -> swerve.flipPoseToRight(swerve.optionalFlipPoseSeededRed(new Pose2d(9.4, 3.64, Rotation2d.fromDegrees(0)))), 0.2, 2.5, 2.5),
              pivot.setDutyCycle(()-> -0.05)
          ))
          .andThen(
            deadline(
              swerve.driveToPoseWithSpeedLimit(() -> swerve.flipPoseToRight(swerve.optionalFlipPoseSeededRed(new Pose2d(10.68, 2.7, Rotation2d.fromDegrees(-60)))), 0.15, 2.0, 0.5),
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
              swerve.driveToPoseWithSpeedLimit(() -> swerve.flipPoseToRight(swerve.optionalFlipPoseSeededRed(new Pose2d(10.84, 2.3, Rotation2d.fromDegrees(90)))), 0.08, 3, 2.5)   //find this pose, right next to bump
              .andThen(swerve.driveToPose(() -> swerve.flipPoseToRight(swerve.optionalFlipPoseSeededRed(new Pose2d(9.87, 1.9, Rotation2d.fromDegrees(167.07)))), 0.1, 3.0)) //find this pose, get ready for pacman
              .andThen(
                  swerve.driveToPoseWithSpeedLimit(()-> swerve.flipPoseToRight(swerve.optionalFlipPoseSeededRed(new Pose2d(9.3, 1.9, Rotation2d.fromDegrees(167.07)))), 0.15, 3.0, 3.0)
              ),
              pivot.goToPositionDegreesWithCondition(pivot.pivotExtendedPositionDegrees, pivot.withinTolerance).andThen(pivot.setDutyCycle(()-> -0.05)),             
              parallel(
                    tripleShooter.setDutyCycle(()-> -0.4),
                    kicker.setDutyCycle(()-> -1)
                  ).withTimeout(3).andThen(parallel(tripleShooter.setDutyCycle(()-> 0.0), kicker.setDutyCycle(()-> 0))),
              feeder.openLoopSet(()-> 0)
            )
          )
          .andThen(
            deadline(
              swerve.driveToPoseWithSpeedLimit(() -> swerve.flipPoseToRight(swerve.optionalFlipPoseSeededRed(new Pose2d(8.15, 3.5, Rotation2d.fromDegrees(77.07)))), 0.08, 2.5, 2.5),
              pivot.setDutyCycle(()-> -0.05)
            )
          )
          .andThen(
            deadline(
              swerve.driveToPose(() -> swerve.flipPoseToRight(swerve.optionalFlipPoseSeededRed(new Pose2d(9.4, 3.64, Rotation2d.fromDegrees(0)))), 0.2, 2.5),
              pivot.setDutyCycle(()-> -0.05)
          ))
          .andThen(
            deadline(
              swerve.driveToPose(() -> swerve.flipPoseToRight(swerve.optionalFlipPoseSeededRed(new Pose2d(10.68, 2.7, Rotation2d.fromDegrees(-60)))), 0.15, 0.5),
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
    intakeRollers.isStalled.onTrue(led.breatheEffect(3, 0.1));
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
    driver.leftBumper().toggleOnTrue(
      led.solidColor(2)
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

    driver.leftTrigger().whileTrue(led.solidColor(5));

    /*
     * Auto shoot while held
     */

    driver.rightTrigger().whileTrue(
      autoShoot()
    );

    /*
     * turn towards hub with custom center - need to change custom center
     */
    // driver.rightTrigger().whileTrue(
    //   swerve.rotateTowardsVirtualHubCustomCenter(
    //     driver::getRequestedChassisSpeeds, 
    //     ()-> shotCalculator.getCachedSetpoint().virtualTarget(),
    //     () -> new Translation2d(
    //         -Constants.Swerve.WHEEL_BASE_METERS/2  * 
    //         -(Math.pow((swerve.getSavedPose().getRotation().plus(Rotation2d.fromDegrees(90))).getCos(), 3) 
    //         - Math.pow((swerve.getSavedPose().getRotation().plus(Rotation2d.fromDegrees(90))).getSin(), 3))
    //         , 
    //         Constants.Swerve.WHEEL_BASE_METERS/2  * 
    //         (Math.pow(swerve.getSavedPose().getRotation().getCos(), 3) 
    //         - Math.pow(swerve.getSavedPose().getRotation().getSin(), 3))
    //       ) 
    //     )
    // );

    driver.rightTrigger().onFalse(
      shooterCooldown()
    );

    // /*
    //  * stockpile while held (raises intake)
    //  */
    // driver.rightBumper().whileTrue(
    //   (
    //     parallel(
    //       tripleShooter.setVelocityTorqueCurrentMPS(()-> shotCalculator.getStockpileSetpoint().shotVelocityMPS()),
    //       swerve.rotateTowardsVirtualHub(driver::getRequestedChassisSpeeds, ()-> shotCalculator.getClosestStockpileTarget())
    //     ).until(tripleShooter.allShootersWithinTolerance)
    //     .andThen(
    //         kicker.setVelocityMPS(()-> kicker.maxSpeedRPS * kicker.surfaceMetersPerMotorRotation).until(kicker.velAboveThreshold)
    //     )
    //     .andThen(
    //       parallel(
    //         swerve.rotateTowardsVirtualHub(driver::getRequestedChassisSpeeds, ()-> shotCalculator.getClosestStockpileTarget()),
    //         kicker.setVelocityMPS(()-> kicker.maxSpeedRPS * kicker.surfaceMetersPerMotorRotation),
    //         tripleShooter.setVelocityTorqueCurrentMPS(()-> shotCalculator.getStockpileSetpoint().shotVelocityMPS()),
    //         feeder.setVelocityMPS(()-> -feeder.metersPerRotationOfMotor * 100),
    //         pivot.slowRaise()
    //       )
    //     )
    //   ).alongWith(
    //     hood.setActuatorDeg(()-> shotCalculator.getStockpileSetpoint().angle())
    //   )
    // );

    /*
     * stockpile while held (does not raise intake)
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
            feeder.setVelocityMPS(()-> -feeder.metersPerRotationOfMotor * 100)
          )
        )
      ).alongWith(
        hood.setActuatorDeg(()-> shotCalculator.getStockpileSetpoint().angle())
      )
    );



    driver.rightBumper().onFalse(
      shooterCooldown()
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
    driver.b().whileTrue(
      cornerShot()
    );

    // driver.povLeft().onTrue(
    //   pivot.setPositionDegrees(()-> pivot.pivotRetractedPositionDegrees)
    // );


    driver.x().whileTrue(
      parallel(
        swerve.rotateTowardsVirtualHub(driver::getRequestedChassisSpeeds, ()-> shotCalculator.getCachedSetpoint().virtualTarget()),
        tripleShooter.setVelocityTorqueCurrentMPS(()-> shotCalculator.getCachedSetpoint().shotVelocityMPS())
      )
    );

    driver.y().whileTrue(
          parallel(
            kicker.setVelocityMPS(()-> kicker.maxSpeedRPS * kicker.surfaceMetersPerMotorRotation),
            tripleShooter.setVelocityTorqueCurrentMPS(()-> shotCalculator.getCachedSetpoint().shotVelocityMPS()),
            feeder.setVelocityMPS(()-> shotCalculator.getCachedSetpoint().feedSpeed()),
            pivot.setDutyCycle(()-> 0.3).until(pivot.pastThresholdAutoRaise).andThen(pivot.setDutyCycle(()-> 0.1)),
            swerve.rotateTowardsVirtualHub(driver::getRequestedChassisSpeeds, ()-> shotCalculator.getCachedSetpoint().virtualTarget())
          )
    );

    // rotates around bottom right corner with custom center
    driver.povRight().whileTrue(
        swerve.driveWithCenterOfRotationCommand(
          driver::getRequestedChassisSpeeds,
          () -> 
           new Translation2d(
            -Constants.Swerve.WHEEL_BASE_METERS/2  * 
            -(Math.pow((swerve.getSavedPose().getRotation().plus(Rotation2d.fromDegrees(90))).getCos(), 3) 
            - Math.pow((swerve.getSavedPose().getRotation().plus(Rotation2d.fromDegrees(90))).getSin(), 3))
            
            , 
            -Constants.Swerve.WHEEL_BASE_METERS/2  * 
            (Math.pow(swerve.getSavedPose().getRotation().getCos(), 3) 
            - Math.pow(swerve.getSavedPose().getRotation().getSin(), 3))
            ) 
           // update with center of rotation
      )
    );

    // rotates around bottom left corner with custom center
    driver.povLeft().whileTrue(
      swerve.driveWithCenterOfRotationCommand(
          driver::getRequestedChassisSpeeds,
          () -> 
           new Translation2d(
            -Constants.Swerve.WHEEL_BASE_METERS/2 * 
            -(Math.pow((swerve.getSavedPose().getRotation().plus(Rotation2d.fromDegrees(90))).getCos(), 3) 
            - Math.pow((swerve.getSavedPose().getRotation().plus(Rotation2d.fromDegrees(90))).getSin(), 3))
            , 
            Constants.Swerve.WHEEL_BASE_METERS/2  * 
            (Math.pow(swerve.getSavedPose().getRotation().getCos(), 3) 
            - Math.pow(swerve.getSavedPose().getRotation().getSin(), 3))
          ) 
           // update with center of rotation
        )
    );
    


    driver.y().onFalse(
      pivot.goToPositionDegreesWithCondition(pivot.pivotRetractedPositionDegrees, pivot.withinTolerance)
      .andThen(
        pivot.setPositionDegrees(()-> pivot.pivotExtendedPositionDegrees)
      )
    );
  }

  public Command shooterCooldown(){
   return  (
        parallel(
          tripleShooter.setDutyCycle(()-> -0.4),
          kicker.setDutyCycle(()-> -1)
        ).withTimeout(0.5)
        .andThen(
          tripleShooter.setDutyCycle(()-> 0)
            .alongWith(kicker.setDutyCycle(()-> 0))
        )
      ).alongWith(pivot.setPositionDegrees(()-> pivot.pivotExtendedPositionDegrees));
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

    public Command cornerShot(){
      return (
        swerve.setAllianceHubTagsValid()
        .andThen(
          parallel(
            tripleShooter.setVelocityTorqueCurrentMPS(()-> 2 * Math.PI * Units.inchesToMeters(2) * 100 * 0.7),
            swerve.rotateToAngle(()-> 
                {
                  if(!swerve.isRedAlliance()){
                    if(swerve.getSavedPose().getY() > Constants.FieldConstants.FIELD_WIDTH/2) return -41;
                    else return 41;
                  }
                  if(swerve.getSavedPose().getY() < Constants.FieldConstants.FIELD_WIDTH/2) return -41;
                  else return 41 + 180;
                }, 
            driver::getRequestedChassisSpeeds)
        ).until(tripleShooter.allShootersWithinTolerance)
        )
        .andThen(
          kicker.setVelocityMPS(()-> kicker.maxSpeedRPS * kicker.surfaceMetersPerMotorRotation).until(kicker.velAboveThreshold)
        )
        .andThen(
          parallel(
            kicker.setVelocityMPS(()-> kicker.maxSpeedRPS * kicker.surfaceMetersPerMotorRotation),
            tripleShooter.setVelocityTorqueCurrentMPS(()-> 2 * Math.PI * Units.inchesToMeters(2) * 100 * 0.7),
            feeder.setVelocityMPS(()-> shotCalculator.getCachedSetpoint().feedSpeed()),
            pivot.slowRaise(),
            // swerve.rotateTowardsVirtualHub(driver::getRequestedChassisSpeeds, ()-> shotCalculator.getCachedSetpoint().virtualTarget())
            swerve.rotateToAngle(()->                 {
                  if(!swerve.isRedAlliance()){
                    if(swerve.getSavedPose().getY() > Constants.FieldConstants.FIELD_WIDTH/2) return -43.18;
                    else return 43.18;
                  }
                  if(swerve.getSavedPose().getY() < Constants.FieldConstants.FIELD_WIDTH/2) return -43.18 + 180;
                  else return 43.18 + 180;
                }, driver::getRequestedChassisSpeeds)
          )
        )
      ).alongWith(hood.setActuatorDeg(()-> 63));
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
        () -> //52
        // 72.5

        shotCalculator.getCachedSetpoint().angle()
      )
    );
    led.setDefaultCommand(led.solidColor(()-> swerve.isRedAlliance() ? 0 : 4).ignoringDisable(true));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}