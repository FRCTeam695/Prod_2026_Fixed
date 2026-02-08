package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.BisonLib.BaseProject.Swerve.SwerveBase;
import frc.BisonLib.BaseProject.Swerve.Modules.TalonFXModule;
import frc.robot.Constants;

import java.util.List;


public class Swerve extends SwerveBase{
    
    public Translation2d targetLocationPose;

    public Pose2d[] reefVerticies = new Pose2d[6];

    //NT
    private final NetworkTableInstance inst;
    private final NetworkTable swerveTable;

    public DoubleArraySubscriber fuelLocationsSub; 
    public DoublePublisher commandedVelocityXPub;
    public DoublePublisher commandedVelocityYPub;
    public BooleanPublisher atDestinationPub;
    public BooleanPublisher fullyAutonomousPub;
    public DoublePublisher distanceToTarget;
    public StringPublisher targetPosePub;
    
    public Trigger isFullyAutonomous;
    public Trigger isAtDestination;
    public Translation2d commandedVelocity;

    public final double kp_attract = 3.5;

    // we will tune this on the practice field
    public final double kp_repulse = 2;

    public boolean currentlyFullyAutonomous = false;


    public Swerve(String[] camNames, TalonFXModule[] modules, int[] reefTags) {
        super(camNames, modules, reefTags);

        inst = NetworkTableInstance.getDefault();
        swerveTable = inst.getTable("Swerve");

        fuelLocationsSub = swerveTable.getDoubleArrayTopic("Fuel Locations").subscribe(new double[0]);
        commandedVelocityXPub = swerveTable.getDoubleTopic("X Commanded Velocity").publish(PubSubOption.periodic(0.02));
        commandedVelocityYPub = swerveTable.getDoubleTopic("Y Commanded Velocity").publish(PubSubOption.periodic(0.02));
        atDestinationPub = swerveTable.getBooleanTopic("At Destination").publish(PubSubOption.periodic(0.02));
        fullyAutonomousPub = swerveTable.getBooleanTopic("Fully Autonomous").publish(PubSubOption.periodic(0.02));
        distanceToTarget = swerveTable.getDoubleTopic("Distance to Target").publish(PubSubOption.periodic(0.02));
        targetPosePub = swerveTable.getStringTopic("Target Pose").publish(PubSubOption.periodic(0.02));

        isFullyAutonomous = new Trigger(()-> currentlyFullyAutonomous);
        isAtDestination = new Trigger(()-> getDistanceToTranslation(targetLocationPose) < 0.02);
        commandedVelocity = new Translation2d();

    }

     public Command viewFuel(Supplier<List<Translation2d>> fuelSupplier){
        return run( 
            () -> {
                List<Translation2d> fuelList = fuelSupplier.get();
                for (int i = 0; i < fuelList.size(); i++){
                    m_field.getObject("fuel " + i).setPose(new Pose2d(fuelList.get(i), new Rotation2d()));
                }
                for (int i = fuelList.size(); i < 50; i++){
                    m_field.getObject("fuel " + i).setPose(new Pose2d(-10,-10, new Rotation2d()));
                }
            });
    }

    public Translation2d getMostEfficientFuelToDriveTo(List<Translation2d> allFuel){
        
        Pose2d robotPose = getSavedPose();
        ChassisSpeeds robotSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(getLatestChassisSpeed(), robotPose.getRotation());

        Translation2d bestFuel = new Translation2d();
        double mostVelocityTowardsFuel = Double.NEGATIVE_INFINITY;

        for (Translation2d fuel : allFuel){
            double dx = fuel.getX() - robotPose.getX();
            double dy = fuel.getY() - robotPose.getY();
            double distanceToFuel = Math.hypot(dx,dy);
                 
            double currentVelocityTowardsFuel = (robotSpeed.vxMetersPerSecond*dx + robotSpeed.vyMetersPerSecond*dy)/distanceToFuel;

            if (currentVelocityTowardsFuel > mostVelocityTowardsFuel){
                mostVelocityTowardsFuel = currentVelocityTowardsFuel;
                bestFuel = fuel;
            }
        }

        return bestFuel;
    }

    public Command driveToBestFuel(Translation2d bestFuel){
        return
            runOnce(() -> currentlyFullyAutonomous = true)
            .andThen(
                run(()->{

                    targetLocationPose = bestFuel;
                    double kP_assist = 3.5;

                    Pose2d robotPose = getSavedPose();
                    ChassisSpeeds robotSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(getLatestChassisSpeed(), robotPose.getRotation());

                    Translation2d robotToFuel = targetLocationPose.minus(robotPose.getTranslation());
                    commandedVelocity = new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);
                    
                    Translation2d direction_commandedVelocity = commandedVelocity.div(commandedVelocity.getNorm());

                    Translation2d robotToFuelParallel = direction_commandedVelocity.times(robotToFuel.dot(direction_commandedVelocity));
                    Translation2d robotToFuelPerpendicular = robotToFuel.minus(robotToFuelParallel);

                    Translation2d newCommandedVel = commandedVelocity.plus(robotToFuelPerpendicular.times(kP_assist));

                    ChassisSpeeds speeds = new ChassisSpeeds(
                        MathUtil.clamp(newCommandedVel.getX(), -Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND, Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND),
                        MathUtil.clamp(newCommandedVel.getY(), -Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND, Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND),
                        getAngularComponentFromRotationOverride(robotPose.getRotation().getDegrees())
                    );

                    drive(speeds, true, false);

                })
            ).until(isAtDestination)
            .andThen(
                runOnce(()->{
                    this.stopModules();
                })
            )
            .finallyDo(()->{
                currentlyFullyAutonomous = false;
            });
    }

    public Command driveForwards(){
        return run(
            ()->{
                driveRobotRelative(new ChassisSpeeds(1, 0, 0), false);
            }
        );
    }

    public Command driveBackwards(){
        return run(()->{
            driveRobotRelative(new ChassisSpeeds(-1, 0, 0), false);
        });
    }

    public Command driveBackwardsRobotRelative(){
        return run(()-> {
            driveRobotRelative(new ChassisSpeeds(-1, 0, 0), false);
            }).withTimeout(0.5);
    }

    public Command leftGyroReset(){
        return resetGyro(90);
    }

    public Command rightGyroReset(){
        return resetGyro(-90);
    }

    @Override
    public void periodic(){
        super.periodic();
        
        commandedVelocityXPub.set(commandedVelocity.getX());
        commandedVelocityYPub.set(commandedVelocity.getY());
        atDestinationPub.set(isAtDestination.getAsBoolean());
        fullyAutonomousPub.set(isFullyAutonomous.getAsBoolean());
        //distanceToTarget.set(getDistanceToTranslation(targetLocationPose));
        targetPosePub.set(targetLocationPose.toString());
        

    }
}

