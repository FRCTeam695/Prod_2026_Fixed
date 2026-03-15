package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.BisonLib.BaseProject.Swerve.SwerveBase;
import frc.BisonLib.BaseProject.Swerve.Modules.TalonFXModule;
import frc.robot.Constants;


public class RebuiltSwerve extends SwerveBase{

    public final double BUMP_THRESHOLD = 1.0;
        
    protected final Field2d m_field = new Field2d();
    
    private final double kp_attract = 2.5;

    public SlewRateLimiter pacmanHeadingFilter = new SlewRateLimiter(Math.toRadians(800.0));




    public RebuiltSwerve(String[] camNames, TalonFXModule[] modules, int[] reefTags) {
        super(camNames, modules, reefTags);
    }

    public Pose2d optionalFlipPose(Pose2d pose){
        if(isRedAlliance()){
            return new Pose2d(Constants.FieldConstants.FIELD_LENGTH - pose.getX(), Constants.FieldConstants.FIELD_WIDTH - pose.getY(), Rotation2d.fromDegrees(pose.getRotation().getDegrees() + 180));
        }
        return pose;
    }

    public Pose2d getOutpostPose(){
        if(isRedAlliance()){
           return optionalFlipPose(new Pose2d(0.655,0.64, Rotation2d.fromDegrees(180)));
        }
        return new Pose2d(0.655 - 0.075 ,0.64, Rotation2d.fromDegrees(180));
    }
    
    public Command rotateTowardsVirtualHub(Supplier<ChassisSpeeds> wantedSpeeds, Supplier<Translation2d> hubPose){
        return 
        run(()->{
            Pose2d virtualPose = new Pose2d(hubPose.get(), new Rotation2d());
            Pose2d robotPose = getSavedPose();
            ChassisSpeeds currentSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(getLatestChassisSpeed(), getSavedPose().getRotation());

            m_field.getObject("virtualHub").setPose(virtualPose);
            SmartDashboard.putString("virtualHub", virtualPose.toString());

            double dx = virtualPose.getX() - robotPose.getX();
            double dy = virtualPose.getY() - robotPose.getY();
            double norm = Math.hypot(dx, dy);

            SmartDashboard.putNumber("distance to hub", norm);

            double theta = Math.toDegrees(Math.atan2(dy, dx));

            double feedForward = (currentSpeeds.vyMetersPerSecond * dx - currentSpeeds.vxMetersPerSecond * dy)
                 / (norm * norm);

            ChassisSpeeds speeds = wantedSpeeds.get();

            speeds.omegaRadiansPerSecond = -feedForward + getAngularComponentFromRotationOverride(theta);

            drive(speeds);


        });
    
    }


    public Command safeTraverseBump(Supplier<ChassisSpeeds> speedSupplier){
        return 

        run(()->{

            double r = getSavedPose().getRotation().getDegrees();

            double smallestError = Double.POSITIVE_INFINITY;

            double bestAngleInDegrees = 45;

            for (double i = 0; i < 4; i++) {
                double wantedAngle = i * 90 + 45;
                double error = Math.abs(MathUtil.inputModulus(wantedAngle - r, -180, 180));
                if (error < smallestError) {
                    smallestError = error;
                    bestAngleInDegrees = wantedAngle;
                }
            }

            ChassisSpeeds speeds = speedSupplier.get();

            speeds.omegaRadiansPerSecond = getAngularComponentFromRotationOverride(bestAngleInDegrees);
            
            drive(speeds);
        });


    }

    public Command pacmanDrive(Supplier<ChassisSpeeds> commandedSpeedsSupplier, Supplier<Translation2d> headingOverrideSupplier){
        return(
            run(()->{
                ChassisSpeeds commandedSpeeds = commandedSpeedsSupplier.get();
                double vx = commandedSpeeds.vxMetersPerSecond;
                double vy = commandedSpeeds.vyMetersPerSecond;

                Translation2d translationOverride = headingOverrideSupplier.get();
                
                double rightStickMagnitude = translationOverride.getNorm();

                double headingOverride = Math.toDegrees(Math.atan2(translationOverride.getY(), translationOverride.getX())) - 90;
                
                double pacmanAngularSpeeds = getAngularComponentFromRotationOverride(Math.toDegrees(Math.atan2(vy, vx)));
                
                // to make sure we don't snap back to theta=0 when not driving
                if(Math.hypot(vx, vy) < 0.05){
                    pacmanAngularSpeeds = commandedSpeeds.omegaRadiansPerSecond;
                }

                // if there is some feedback from the right stick, snap to that angle (heading override)
                if(rightStickMagnitude >= 0.05){
                    pacmanAngularSpeeds = getAngularComponentFromRotationOverride(headingOverride);
                }

                // pacmanAngularSpeeds = pacmanHeadingFilter.calculate(pacmanAngularSpeeds);

                ChassisSpeeds adjustedSpeeds = new ChassisSpeeds(vx, vy, pacmanAngularSpeeds);

                drive(adjustedSpeeds);

            })
        );
    }


    public Command driveToPose(Supplier<Pose2d> targetPoseSupplier, double distanceEnd, double feedForward){

        return
            
            run(
            ()->{
                ChassisSpeeds speeds = getNextSpeedsOutput(targetPoseSupplier.get(), getSavedPose(), feedForward);

                drive(speeds);
            }
            ).until(() -> getDistanceToTranslation(targetPoseSupplier.get().getTranslation()) < distanceEnd)
            .andThen(runOnce(()-> {
                SmartDashboard.putBoolean("reached destination", true);
                this.stopModules();
            }));
    }

    public Command pacmanDriveToPose(Supplier<Pose2d> targetPoseSupplier, double distanceEnd, double feedForward){
        return
            
            run(
            ()->{
                Pose2d targetPose = targetPoseSupplier.get();
                Pose2d currentPose = getSavedPose();
                double dx = targetPose.getX() - currentPose.getX();
                double dy = targetPose.getY() - currentPose.getY();
                ChassisSpeeds speeds = getNextSpeedsOutput(targetPose, currentPose, feedForward);
                    
                speeds.omegaRadiansPerSecond = getAngularComponentFromRotationOverride(Math.toDegrees(Math.atan2(dy, dx)));
                SmartDashboard.putString("align speeds", speeds.toString());

                drive(speeds);
            }
            ).until(() -> getDistanceToTranslation(targetPoseSupplier.get().getTranslation()) < distanceEnd)
            .andThen(runOnce(()-> {
                SmartDashboard.putBoolean("reached destination", true);
                this.stopModules();
            }));
    }

    public Command driveToPoseWhileTurningToHub(Supplier<Pose2d> drivePoseSupplier, double distanceEnd){

        return 
            
            run(()->{
                Pose2d robotPose = getSavedPose();
                ChassisSpeeds currentSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(getLatestChassisSpeed(), getSavedPose().getRotation());
                Pose2d targetTurnPose = isRedAlliance() ? Constants.FieldConstants.Red.HUB : Constants.FieldConstants.Blue.HUB;

                double dx_turn = targetTurnPose.getX() - robotPose.getX();
                double dy_turn = targetTurnPose.getY() - robotPose.getY();

                double norm = Math.hypot(dx_turn, dy_turn);

                SmartDashboard.putNumber("distance to hub", norm);

                double theta = Math.toDegrees(Math.atan2(dy_turn, dx_turn));

                double feedForward = (currentSpeeds.vyMetersPerSecond * dx_turn - currentSpeeds.vxMetersPerSecond * dy_turn)
                    / (norm * norm);
                
                ChassisSpeeds speeds = getNextSpeedsOutput(drivePoseSupplier.get(), robotPose, 0);

                speeds.omegaRadiansPerSecond = -feedForward + getAngularComponentFromRotationOverride(theta);

                drive(speeds);
                
            }
            ).until(() -> getDistanceToTranslation(drivePoseSupplier.get().getTranslation()) < distanceEnd)
            .andThen(runOnce(()-> {
                SmartDashboard.putBoolean("reached destination", true);
                this.stopModules();
            }));
    }

    public ChassisSpeeds getNextSpeedsOutput(Pose2d targetPose, Pose2d currentPose, double feedforward){
                SmartDashboard.putBoolean("reached destination", false);
 
                m_field.getObject("targetPose").setPose(targetPose);
                SmartDashboard.putString("targetPose", targetPose.toString());

                // the current field relative robot pose
                Pose2d robotPose = getSavedPose();

                double dx = targetPose.getX() - robotPose.getX();
                double dy = targetPose.getY() - robotPose.getY();

                // converting the errors to components of a unit vector
                double distance = Math.hypot(dx, dy);
                double unitX = dx / distance;
                double unitY = dy / distance;

                SmartDashboard.putNumber("alignment dx", dx);
                SmartDashboard.putNumber("alignment dy", dy);

                double speed = MathUtil.clamp(kp_attract * distance + feedforward, 
                    -Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS_TELEOP, 
                    Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS_TELEOP);


                // makes robot go straight by applying calculated velocity to unit vector
                double attractY = unitY * speed;
                double attractX = unitX * speed;
           
                // SmartDashboard.putNumber("desired velocity", desiredVelocity);
                SmartDashboard.putNumber("distance to target", distance);
                SmartDashboard.putNumber("attract speed", Math.hypot(attractX, attractY));
                
                ChassisSpeeds speeds =
                    new ChassisSpeeds(
                        MathUtil.clamp(attractX, -Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS_TELEOP, Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS_TELEOP),
                        MathUtil.clamp(attractY, -Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS_TELEOP, Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS_TELEOP),
                    getAngularComponentFromRotationOverride(targetPose.getRotation().getDegrees())
                );

                return speeds;
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putData("Field", m_field);

         m_field.getObject("robot pose").setPose(getSavedPose());
        SmartDashboard.putString("robot pose", getSavedPose().toString());

        SmartDashboard.putNumber("robot rotation", getSavedPose().getRotation().getDegrees());
    }
}