// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.BisonLib.BaseProject.Controller.EnhancedCommandController;
import frc.BisonLib.BaseProject.Swerve.Modules.TalonFXModule;
import frc.BisonLib.BaseProject.Util.ShooterInterpolationMap;
import frc.BisonLib.BaseProject.Util.ShooterInterpolationMap.ShooterSetpoint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import frc.robot.subsystems.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public final Swerve Swerve;
  //public final SwerveBase SwerveSubsystem;
  public IntegerSubscriber scoringHeight;
  SendableChooser<Command> autoChooser = new SendableChooser<>();
  ShooterInterpolationMap shooterInterpolationMap;

  public int[] reefTags = {6,7,8,9,10,11,17,18,19,20,21,22};

  private final TalonFXModule[] modules = new TalonFXModule[]
  {
    new TalonFXModule(Constants.Swerve.FRONT_RIGHT_DRIVE_ID, Constants.Swerve.FRONT_RIGHT_TURN_ID, Constants.Swerve.FRONT_RIGHT_ABS_ENCODER_OFFSET_ROTATIONS, Constants.Swerve.FRONT_RIGHT_CANCODER_ID, 0),
    new TalonFXModule(Constants.Swerve.FRONT_LEFT_DRIVE_ID, Constants.Swerve.FRONT_LEFT_TURN_ID, Constants.Swerve.FRONT_LEFT_ABS_ENCODER_OFFSET_ROTATIONS, Constants.Swerve.FRONT_LEFT_CANCODER_ID, 1),
    new TalonFXModule(Constants.Swerve.BACK_LEFT_DRIVE_ID, Constants.Swerve.BACK_LEFT_TURN_ID, Constants.Swerve.BACK_LEFT_ABS_ENCODER_OFFSET_ROTATIONS, Constants.Swerve.BACK_LEFT_CANCODER_ID, 2),
    new TalonFXModule(Constants.Swerve.BACK_RIGHT_DRIVE_ID, Constants.Swerve.BACK_RIGHT_TURN_ID, Constants.Swerve.BACK_RIGHT_ABS_ENCODER_OFFSET_ROTATIONS, Constants.Swerve.BACK_RIGHT_CANCODER_ID, 3),
  };


  public final VisionManager VisionManager;
  private final String[] limelightCameraNames = {"limelight-left", "limelight-right"};
  private final String[] photonVisionCameraNames= {"intake"};


  private static final EnhancedCommandController driver = new EnhancedCommandController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Swerve = new Swerve(limelightCameraNames, modules, reefTags);
    VisionManager = new VisionManager(photonVisionCameraNames, () -> Swerve.getSavedPose());
   
    scoringHeight = NetworkTableInstance.getDefault().getTable("sidecarTable").getIntegerTopic("scoringLevel").subscribe(1);

    // SmartDashboarding subsystems allow you to see what commands they are running
    SmartDashboard.putData("Swerve Subsystem", Swerve);
    shooterInterpolationMap = new ShooterInterpolationMap("simulated_optimal_trajectories.csv");

    // Configure the trigger bindings
    configureBindings();

    SmartDashboard.putData(autoChooser);

    DataLogManager.start();
  }
  
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    
    driver.back().and(driver.start()).onTrue(Swerve.resetGyro());

    // driver.leftTrigger().whileTrue(
    //   Swerve.driveToBestFuel(
    //     Swerve.getMostEfficientFuelToDriveTo(
    //       VisionManager.getFuelList()
    //     )
    //   )
    // );
  }

  public void configureDefaultCommands(){
    // This is the Swerve subsystem default command, this allows the driver to drive the robot
    Swerve.setDefaultCommand
      (
        
        run
          (
            ()-> 
              Swerve.teleopDefaultCommand(
                driver::getRequestedChassisSpeeds,
                true
              )
              ,
              Swerve
          ).withName("Swerve Drive Command"))
      ;

    // driver.a().onTrue(putInterpolatedShooterSetpointsToNetworktables(()-> 1.5, ()-> 0.0918)); // 72.1278,2296.06
    // driver.b().onTrue(putInterpolatedShooterSetpointsToNetworktables(()-> 2.2627, ()-> 1.7449)); // 81.6115,2326.14
    // driver.x().onTrue(putInterpolatedShooterSetpointsToNetworktables(()-> 3.1102, ()-> 3.0306)); // 86.2431, 2362.77
    // driver.x().onTrue(putInterpolatedShooterSetpointsToNetworktables(()-> 5.1441, ()-> -0.0918)); // 53.9323,3200.07
  }

  public Runnable getOdometryUpdater(){
    return Swerve::updateOdometryWithKinematics;
  }

  private Command putInterpolatedShooterSetpointsToNetworktables(DoubleSupplier d, DoubleSupplier v){
    return runOnce(()-> {
      ShooterSetpoint s = shooterInterpolationMap.getSetpoint(d.getAsDouble(), v.getAsDouble());
      SmartDashboard.putNumber("Setpoint RPM", s.rpm());
      SmartDashboard.putNumber("Setpoint Angle", s.angle());
    });
  }

  // The command specified in here is run in autonomous
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}