package frc.robot.subsystems;



import static edu.wpi.first.units.Units.Rotation;

import java.security.spec.ECPublicKeySpec;
import java.util.List;
import java.util.Optional;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.BisonLib.BaseProject.Swerve.SwerveBase;
import frc.BisonLib.BaseProject.Swerve.Modules.TalonFXModule;
import frc.robot.Constants;

public class Swerve extends SwerveBase {

    public Translation2d targetLocationTranslation;

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
    public boolean isAtDestination;
    public Trigger bestFuelPresent;
    public Translation2d commandedVelocity;

    public boolean currentlyFullyAutonomous = false;

    public Swerve(String[] camNames, TalonFXModule[] modules, int[] reefTags) {
        super(camNames, modules, reefTags);

          inst = NetworkTableInstance.getDefault();
        swerveTable = inst.getTable("Swerve");
        targetLocationTranslation = new Translation2d(100,0);

        fuelLocationsSub = swerveTable.getDoubleArrayTopic("Fuel Locations").subscribe(new double[0]);
        commandedVelocityXPub = swerveTable.getDoubleTopic("X Commanded Velocity").publish(PubSubOption.periodic(0.02));
        commandedVelocityYPub = swerveTable.getDoubleTopic("Y Commanded Velocity").publish(PubSubOption.periodic(0.02));
        atDestinationPub = swerveTable.getBooleanTopic("At Destination").publish(PubSubOption.periodic(0.02));
        fullyAutonomousPub = swerveTable.getBooleanTopic("Fully Autonomous").publish(PubSubOption.periodic(0.02));
        distanceToTarget = swerveTable.getDoubleTopic("Distance to Target").publish(PubSubOption.periodic(0.02));
        targetPosePub = swerveTable.getStringTopic("Target Pose").publish(PubSubOption.periodic(0.02));

        isFullyAutonomous = new Trigger(()-> currentlyFullyAutonomous);
        isAtDestination = false;
        commandedVelocity = new Translation2d(0,0);
        }

    public boolean isOnBump() {
        double pitch = Math.abs(pigeon.getPitch().getValueAsDouble());
        double roll = Math.abs(pigeon.getRoll().getValueAsDouble());

        System.out.println("pitch = " + pitch);
        System.out.println("roll = " + roll);

        return pitch > Constants.BUMP_THRESHOLD || roll > Constants.BUMP_THRESHOLD;
    }

    public Command bumpTest() {
        return runOnce(() -> {
            if (isOnBump()) {
                System.out.println("ON BUMP");
            }
        });
    }

     public Command viewFuel(Supplier<List<Translation2d>> fuelSupplier,Supplier<Optional<Translation2d>> closestFuel){
        return run( 
            () -> {
                List<Translation2d> fuelList = fuelSupplier.get();
                for (int i = 0; i < fuelList.size(); i++){
                    m_field.getObject("fuel " + i).setPose(new Pose2d(fuelList.get(i), new Rotation2d()));
                }
                for (int i = fuelList.size(); i < 50; i++){
                    m_field.getObject("fuel " + i).setPose(new Pose2d(-10,-10, new Rotation2d()));
                }

                if(closestFuel.get().isPresent()){
                    m_field.getObject("ClosestFuel").setPose(new Pose2d(closestFuel.get().get(), new Rotation2d(Math.PI/2)));
                }else{
                    m_field.getObject("ClosestFuel").setPose(new Pose2d(-10,-10, new Rotation2d()));
                }
        
            }).ignoringDisable(true);
    }

    public Optional<Translation2d> getMostEfficientFuelToDriveTo(Supplier<List<Translation2d>> allFuelSupplier){
        
        List<Translation2d> allFuelList = allFuelSupplier.get();

        Pose2d robotPose = getSavedPose();
        ChassisSpeeds robotSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(getLatestChassisSpeed(), robotPose.getRotation());

        Translation2d bestFuel = new Translation2d();
        double mostVelocityTowardsFuel = Double.NEGATIVE_INFINITY;

        if (allFuelList.size() != 0){
            for (Translation2d fuel : allFuelList){
            double dx = fuel.getX() - robotPose.getX();
            double dy = fuel.getY() - robotPose.getY();
            double distanceToFuel = Math.hypot(dx,dy);
                 
            double currentVelocityTowardsFuel = (robotSpeed.vxMetersPerSecond*dx + robotSpeed.vyMetersPerSecond*dy)/distanceToFuel;

                if (currentVelocityTowardsFuel > mostVelocityTowardsFuel){
                    mostVelocityTowardsFuel = currentVelocityTowardsFuel;
                    bestFuel = fuel;
                }
            }
            return Optional.of(bestFuel);
        }
        
        return Optional.empty();

    }

     public Command driveToBestFuel(Supplier<Optional<Translation2d>> bestFuelSupplier){
        return
            runOnce(() -> currentlyFullyAutonomous = true)
            .andThen(
                runOnce( ()-> {System.out.println(bestFuelSupplier.get().isEmpty());})
            )
            .andThen(
                run(()->{
                    Optional<Translation2d> bestFuel = bestFuelSupplier.get();
                    System.out.println(bestFuel.isEmpty());
                    if (bestFuel.isPresent()){

                        isAtDestination = false;

                        // System.out.println("driving");

                        targetLocationTranslation = bestFuel.get();

                        double kP_assist = 1;

                        Pose2d robotPose = getSavedPose();
                        ChassisSpeeds robotSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(getLatestChassisSpeed(), robotPose.getRotation());

                        Translation2d robotToFuel = targetLocationTranslation.minus(robotPose.getTranslation());
                                                
                        Translation2d direction_commandedVelocity;

                        if (commandedVelocity.getNorm() <= 0.01){
                            direction_commandedVelocity = new Translation2d(0,0);
                        } else {
                            direction_commandedVelocity = commandedVelocity.div(commandedVelocity.getNorm());
                        }

                        Translation2d robotToFuelParallel = direction_commandedVelocity.times(robotToFuel.dot(direction_commandedVelocity));
                        Translation2d robotToFuelPerpendicular = robotToFuel.minus(robotToFuelParallel);

                        Translation2d newCommandedVel = commandedVelocity.plus(robotToFuelPerpendicular.times(kP_assist));
                        SmartDashboard.putString("STUPID TRANSLATION", newCommandedVel.toString());

                        ChassisSpeeds speeds = new ChassisSpeeds(
                            MathUtil.clamp(newCommandedVel.getX(), -Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND, Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND),
                            MathUtil.clamp(newCommandedVel.getY(), -Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND, Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND),
                            getAngularComponentFromRotationOverride(robotPose.getRotation().getDegrees())
                        );

                        drive(speeds, true, false);
                    }
                })
            ).until(new Trigger(()-> getDistanceToTranslation(targetLocationTranslation) < 0.02).or(()-> bestFuelSupplier.get().isEmpty()))
            .andThen(
                runOnce(()->{
                    isAtDestination = false;
                    this.stopModules();
                    System.out.print("isAtDestination"); System.out.println(isAtDestination);

                    
                })
            )
            .finallyDo(()->{
                currentlyFullyAutonomous = false;
            });
     }

     public Command turnTowardsClosestFuel(Supplier<Optional<Translation2d>> closestFuelSupplier, Supplier<ChassisSpeeds> wantedSpeedSupplier){
        return 
            run(() -> {
                Optional<Translation2d> closestFuelOptional = closestFuelSupplier.get();
                if(closestFuelOptional.isPresent()){
                    Translation2d closestFuel= closestFuelOptional.get();
                    Translation2d robotTranslation = getSavedPose().getTranslation();
                    
                    double dx = closestFuel.getX() - robotTranslation.getX();
                    double dy = closestFuel.getY() - robotTranslation.getY();
                    double theta = Math.toDegrees(Math.atan2(dy,dx));

                    ChassisSpeeds speeds = wantedSpeedSupplier.get();
                    speeds.omegaRadiansPerSecond = getAngularComponentFromRotationOverride(theta);
                    drive(speeds,true,true);
                } else{
                    drive(wantedSpeedSupplier.get(),true,true);
                }
            }
            );
     }

      public void periodic(){
        super.periodic();
        
        commandedVelocity = new Translation2d(
            ChassisSpeeds.fromRobotRelativeSpeeds(getLatestChassisSpeed(), getSavedPose().getRotation()).vxMetersPerSecond,
            ChassisSpeeds.fromRobotRelativeSpeeds(getLatestChassisSpeed(), getSavedPose().getRotation()).vyMetersPerSecond);

        commandedVelocityXPub.set(commandedVelocity.getX());
        commandedVelocityYPub.set(commandedVelocity.getY());
        atDestinationPub.set(isAtDestination);
        fullyAutonomousPub.set(isFullyAutonomous.getAsBoolean());
        distanceToTarget.set(getDistanceToTranslation(targetLocationTranslation));
        targetPosePub.set(targetLocationTranslation.toString());
      }
}
