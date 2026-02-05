package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.BisonLib.BaseProject.Swerve.SwerveBase;
import frc.BisonLib.BaseProject.Swerve.Modules.TalonFXModule;


public class RebuiltSwerve extends SwerveBase{
    
    public RebuiltSwerve(String[] camNames, TalonFXModule[] modules, int[] reefTags) {
        super(camNames, modules, reefTags);
    }


    public Command rotateTowardsVirtualHub(Supplier<ChassisSpeeds> wantedSpeeds){
        return run(()->{
            Pose2d virtualPose = new Pose2d();
            Pose2d robotPose = getSavedPose();
            ChassisSpeeds currentSpeeds = getLatestChassisSpeed();
            Twist2d currentTwist = currentSpeeds.toTwist2d(0.02);
            robotPose.plus(new Transform2d(currentTwist.dx, currentTwist.dy, new Rotation2d(currentTwist.dtheta)));

            double dx = virtualPose.getX() - robotPose.getX();
            double dy = virtualPose.getY() - robotPose.getY();
            double theta = Math.toDegrees(Math.atan2(dy, dx));

            ChassisSpeeds speeds = wantedSpeeds.get();
            speeds.omegaRadiansPerSecond = getAngularComponentFromRotationOverride(theta);
            drive(speeds, true, true);
        });
    }

}