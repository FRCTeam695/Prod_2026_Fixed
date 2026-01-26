package frc.robot.subsystems;

import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.BisonLib.BaseProject.Swerve.SwerveBase;
import frc.BisonLib.BaseProject.Swerve.Modules.TalonFXModule;
import frc.robot.Constants;


public class Swerve extends SwerveBase{
    
    public Pose2d targetLocationPose;

    public Pose2d[] reefVerticies = new Pose2d[6];

    //NT
    public NetworkTableInstance inst;
    public NetworkTable sideCarTable;
    public StringSubscriber scoringLocationSub; 
    public StringSubscriber scoringModeSub;
    
    public Trigger isFullyAutonomous;
    public Trigger isAtDestination;

    public final double kp_attract = 3.5;

    // we will tune this on the practice field
    public final double kp_repulse = 2;

    public boolean currentlyFullyAutonomous = false;


    public Swerve(String[] camNames, TalonFXModule[] modules, int[] reefTags) {
        super(camNames, modules, reefTags);

        isFullyAutonomous = new Trigger(()-> currentlyFullyAutonomous);
        isAtDestination = new Trigger(()-> getDistanceToTranslation(targetLocationPose.getTranslation()) < 0.02);

    }
    
    public Pose2d getNearestFuel(){
        return new Pose2d();
    }

    public Pose2d getMostEfficientFuelToDriveTo(Pose2d [] allFuel){
        
        Pose2d robotPose = getSavedPose();
        ChassisSpeeds robotSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(getLatestChassisSpeed(), robotPose.getRotation());

        Pose2d bestFuel = new Pose2d();
        double mostVelocityTowardsFuel = Double.NEGATIVE_INFINITY;

        for (Pose2d fuel : allFuel){
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

    public Command driveToBestFuel(Pose2d bestFuel){
        return
            runOnce(() -> currentlyFullyAutonomous = true)
            .andThen(
                run(()->{

                    double kP_assist = 3.5;

                    Pose2d robotPose = getSavedPose();
                    ChassisSpeeds robotSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(getLatestChassisSpeed(), robotPose.getRotation());

                    Translation2d robotToFuel = bestFuel.getTranslation().minus(robotPose.getTranslation());
                    Translation2d commandedVelocity = new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);
                    
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

    //CAN BE ADAPTED TO HAVE ROBOT ROTATE TO MOST IDEAL PATCH OF FUEL


    // public Command rotateToReefCenter(Supplier<ChassisSpeeds> wantedSpeeds){
    //     return run(()->{
    //         Pose2d reefCenter;
    //         Pose2d robotPose = getSavedPose();
    //         if(isRedAlliance()){
    //             reefCenter = Constants.Vision.Red.REEF_CENTER;
    //         }
    //         else{
    //             reefCenter = Constants.Vision.Blue.REEF_CENTER;
    //         }
    //         targetLocationPose = reefCenter;
    //         double dx = reefCenter.getX() - robotPose.getX();
    //         double dy = reefCenter.getY() - robotPose.getY();
    //         double theta = Math.toDegrees(Math.atan2(dy, dx));
    //         SmartDashboard.putNumber("Desired Robot Rotation", theta);
    //         ChassisSpeeds speeds = wantedSpeeds.get();
    //         speeds.omegaRadiansPerSecond = getAngularComponentFromRotationOverride(theta);
    //         drive(speeds, true, true);
    //     });
    // }


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


    public Transform2d getRepulsionVector(Pose2d robotPose, double repulsionGain){
        currentlyFullyAutonomous = true;
        double repulsionX = 0;
        double repulsionY = 0;
        for(var vertex : reefVerticies){
            ++inc;
            // get distance to vertex
            Transform2d transformToVertex = vertex.minus(robotPose);
            double vdx = vertex.getX() - robotPose.getX();
            double vdy = vertex.getY() - robotPose.getY();
            double distance = transformToVertex.getTranslation().getNorm();
            
            // magnitude of repulsive force
            double f_mag = repulsionGain/Math.pow(distance,2);

            // unit vector of repulsive force
            double unit_vector_x = vdx/distance;
            double unit_vector_y = vdy/distance;

            // multiply by unit vector to get direction and magnitude
            double f_x = f_mag * unit_vector_x;
            double f_y = f_mag * unit_vector_y;

            repulsionX += f_x;
            repulsionY += f_y;
        }

        return new Transform2d(repulsionX, repulsionY, new Rotation2d());
    }


    @Override
    public void periodic(){
        super.periodic();
        m_field.getObject("target location").setPose(targetLocationPose);
        SmartDashboard.putBoolean("At Destination", isAtDestination.getAsBoolean());
        SmartDashboard.putBoolean("Fully Autonomous", isFullyAutonomous.getAsBoolean());
        SmartDashboard.putNumber("Distance to target", getDistanceToTranslation(targetLocationPose.getTranslation()));
    }
}