package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;

import frc.BisonLib.BaseProject.Swerve.SwerveBase;
import frc.BisonLib.BaseProject.Swerve.Modules.TalonFXModule;
import frc.robot.Constants;


public class RebuiltSwerve extends SwerveBase{
    
    private final ProfiledPIDController turnController = new ProfiledPIDController(0.01, 0, 0, new TrapezoidProfile.Constraints(700, 1000));
    private final PIDController wackyController = new PIDController(0.008, 0, 0);

    public RebuiltSwerve(String[] camNames, TalonFXModule[] modules, int[] reefTags) {
        super(camNames, modules, reefTags);
        turnController.enableContinuousInput(-180, 180);
        wackyController.enableContinuousInput(-180, 180);
    }


    public Command rotateTowardsVirtualHub(Supplier<ChassisSpeeds> wantedSpeeds){
        return 
        runOnce(
            ()-> {
                Pose2d robotPose = getSavedPose();
                ChassisSpeeds latestSpeeds = getLatestChassisSpeed();
                turnController.reset(robotPose.getRotation().getDegrees(), Math.toDegrees(latestSpeeds.omegaRadiansPerSecond));
            }
        ).andThen(
        run(()->{
            Pose2d virtualPose = isRedAlliance() ? Constants.FieldConstants.Red.hub : Constants.FieldConstants.Blue.hub;
            Pose2d robotPose = getSavedPose();
            ChassisSpeeds currentSpeeds = getLatestChassisSpeed();
            Twist2d currentTwist = currentSpeeds.toTwist2d(0.8);
            Pose2d wackyPose = robotPose.plus(new Transform2d(currentTwist.dx, currentTwist.dy, new Rotation2d(currentTwist.dtheta)));

            double dx = virtualPose.getX() - robotPose.getX();
            double dy = virtualPose.getY() - robotPose.getY();
            double theta = Math.toDegrees(Math.atan2(dy, dx));
            double thetaWacky = Math.toDegrees(Math.atan2(virtualPose.getY() - wackyPose.getY(), virtualPose.getX() - wackyPose.getX()));
            double thetaDiff = thetaWacky - theta;

            ChassisSpeeds speeds = wantedSpeeds.get();
            double pidOutput = turnController.calculate(robotPose.getRotation().getDegrees(), new State(
                theta, currentSpeeds.omegaRadiansPerSecond
            ));

            getAngularComponentFromRotationOverride(theta); // this line is currently just for logging purposes

            speeds.omegaRadiansPerSecond = -wackyController.calculate(thetaDiff) + Math.toRadians(turnController.getSetpoint().velocity) + pidOutput;
            drive(speeds, true, true);
        }));
    }

}