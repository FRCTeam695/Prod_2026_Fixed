package frc.robot.subsystems;



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
    public boolean fuelDetected;
    public Trigger bestFuelPresent;
    public Translation2d commandedVelocity;

    Optional<Translation2d> bestFuel;
    Optional<Translation2d> lastBestFuel;
    public double distanceFromIntakeToFuel;

    public boolean currentlyFullyAutonomous = false;

    public Swerve(String[] camNames, TalonFXModule[] modules, int[] reefTags) {
        super(camNames, modules, reefTags);

        bestFuel = Optional.empty();
        lastBestFuel = Optional.empty();

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
            }).ignoringDisable(true);
    }

    public Optional<Translation2d> getMostEfficientFuelToDriveTo(Supplier<List<Translation2d>> allFuelSupplier){
        
        List<Translation2d> allFuelList = allFuelSupplier.get();

        Pose2d robotPose = getSavedPose();
        ChassisSpeeds robotSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(getLatestChassisSpeed(), robotPose.getRotation());

        Translation2d bestFuel = new Translation2d();
        // double mostVelocityTowardsFuel = Double.NEGATIVE_INFINITY;

        double fastestTimeToFuel = Double.POSITIVE_INFINITY; // the bigger the index, the better

        if (allFuelList.size() != 0){
            for (Translation2d fuel : allFuelList){
                double dx = fuel.getX() - robotPose.getX();
                double dy = fuel.getY() - robotPose.getY();
                double distanceToFuel = Math.hypot(dx,dy);
                    
                //distance/velcoty 
                // less number would be better

                double currentVelocityTowardsFuel = (robotSpeed.vxMetersPerSecond*dx + robotSpeed.vyMetersPerSecond*dy)/distanceToFuel;

                if (Math.abs(currentVelocityTowardsFuel) < 0.02){
                    currentVelocityTowardsFuel = 1;
                } 

                double timeToFuel;
        
                timeToFuel = distanceToFuel/currentVelocityTowardsFuel;

                if (timeToFuel < fastestTimeToFuel){
                    fastestTimeToFuel = timeToFuel;
                    bestFuel = fuel;
                }
            }
            return Optional.of(bestFuel);
        }
        
        return Optional.empty();

    }

    public Command driveToBestFuel(Supplier<Optional<Translation2d>> bestFuelSupplier, Supplier<List<Translation2d>> fuelListSupplier){
        return
            runOnce(() -> {
                
                currentlyFullyAutonomous = true;
                
                bestFuel = Optional.empty();
                lastBestFuel = Optional.empty();
                
                // saves the fuel pose, uses it all throughout command
                // risky if fuel is far away because it is not updating

                
            })
            .andThen(
                run(()->{

                    bestFuel = bestFuelSupplier.get();

                    viewFuel(fuelListSupplier);
                
                    if (bestFuel.isPresent()){
                        lastBestFuel = bestFuel;
                    }

                    if ( lastBestFuel.isPresent()){
                        fuelDetected = true;
                        isAtDestination = false;

                        targetLocationTranslation = lastBestFuel.get();
                        m_field.getObject("lastBestFuel").setPose(new Pose2d(targetLocationTranslation, new Rotation2d(-Math.PI)));

                        double kp_attract = 1;
        
                        // the current field relative robot pose
                        Translation2d robotPose = getSavedPose().getTranslation();

                        // ChassisSpeeds robotSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(getLatestChassisSpeed(), robotPose.getRotation());

                        Translation2d robotToFuel = targetLocationTranslation.minus(robotPose);

                        double dx = robotToFuel.getX();
                        double dy = robotToFuel.getY();

                        double thetaFuel = (Math.toDegrees(Math.atan2(dy, dx)));

                        SmartDashboard.putNumber("thetaFuel", thetaFuel);

                        // converting the errors to components of a unit vector
                        distanceFromIntakeToFuel = Math.hypot(dx, dy) - 0.4826;

                        Translation2d intakeToFuel = new Translation2d(
                            Math.min(
                                Math.abs(kp_attract * distanceFromIntakeToFuel), 
                                Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS_TELEOP
                            ), 
                                Rotation2d.fromDegrees(thetaFuel)
                            );
                        
  
                
                        // SmartDashboard.putNumber("desired velocity", desiredVelocity);
                        SmartDashboard.putNumber("distance from intake to fuel", distanceFromIntakeToFuel);
                        SmartDashboard.putNumber("attract speed", intakeToFuel.getNorm());
                        
                        ChassisSpeeds speeds =
                            new ChassisSpeeds(
                                MathUtil.clamp(intakeToFuel.getX(), -Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS_TELEOP, Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS_TELEOP),
                                MathUtil.clamp(intakeToFuel.getY(), -Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS_TELEOP, Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS_TELEOP),
                            getAngularComponentFromRotationOverride(thetaFuel)
                        );
                        SmartDashboard.putString("align speeds", speeds.toString());

                        drive(speeds, true, false);
                    
                    } else {
                        fuelDetected = false;
                    }
                })
            .finallyDo(()->{
                currentlyFullyAutonomous = false;
            }));
    }

    public void periodic(){
        super.periodic();
        
        commandedVelocity = new Translation2d(
            ChassisSpeeds.fromRobotRelativeSpeeds(getLatestChassisSpeed(), getSavedPose().getRotation()).vxMetersPerSecond,
            ChassisSpeeds.fromRobotRelativeSpeeds(getLatestChassisSpeed(), getSavedPose().getRotation()).vyMetersPerSecond
        );
        
        commandedVelocityXPub.set(commandedVelocity.getX());
        commandedVelocityYPub.set(commandedVelocity.getY());
        atDestinationPub.set(isAtDestination);
        fullyAutonomousPub.set(isFullyAutonomous.getAsBoolean());
        distanceToTarget.set(getDistanceToTranslation(targetLocationTranslation));
        targetPosePub.set(targetLocationTranslation.toString());
    }
}
