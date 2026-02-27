package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.BisonLib.BaseProject.Swerve.SwerveBase;
import frc.BisonLib.BaseProject.Swerve.Modules.TalonFXModule;


public class RebuiltSwerve extends SwerveBase{

    public final double BUMP_THRESHOLD = 1.0;
    
    private final PIDController turnToHubController = new PIDController(0.06, 0, 0);

    private final PIDController turnToAngleController = new PIDController(0.06, 0, 0);


    public final Trigger onBump;
    
    protected final Field2d m_field = new Field2d();

    public double bestAngle = 0;
    
    // bump constants
    private final double blueLowerX = 0;
    private final double blueUpperX = 0;
    private final double blueLowerY = 0;
    private final double blueUpperY = 0;

    private final double redLowerX = 0;
    private final double redUpperX = 0;
    private final double redLowerY = 0;
    private final double redUpperY = 0;

    public RebuiltSwerve(String[] camNames, TalonFXModule[] modules, int[] reefTags) {
        super(camNames, modules, reefTags);
        turnToAngleController.enableContinuousInput(-180, 180);

        turnToHubController.enableContinuousInput(-180, 180);

        onBump = new Trigger(
            () -> (isOnBumpWithAngle() && isOnBumpWithLocation())
        );

    }


    public Command rotateTowardsVirtualHub(Supplier<ChassisSpeeds> wantedSpeeds, Supplier<Pose2d> hubPose){
        return 
        runOnce(
            ()-> {
                turnToHubController.reset();
            }
        ).andThen(
        run(()->{
            Pose2d virtualPose = hubPose.get();
            Pose2d robotPose = getSavedPose();
            ChassisSpeeds currentSpeeds = getLatestChassisSpeed();

            m_field.getObject("virtualHub").setPose(virtualPose);
            SmartDashboard.putString("virtualHub", virtualPose.toString());

            double dx = virtualPose.getX() - robotPose.getX();
            double dy = virtualPose.getY() - robotPose.getY();
            double norm = Math.hypot(dx, dy);
            double theta = Math.toDegrees(Math.atan2(dy, dx));

            double feedForward = (currentSpeeds.vyMetersPerSecond * dx - currentSpeeds.vxMetersPerSecond * dy)
                 / (norm * norm);

            ChassisSpeeds speeds = wantedSpeeds.get();

            double pidOutput = turnToHubController.calculate(robotPose.getRotation().getDegrees(), theta);

            getAngularComponentFromRotationOverride(theta); // this line is currently just for logging purposes

            speeds.omegaRadiansPerSecond = -feedForward + getAngularComponentFromRotationOverride(theta) + 
            pidOutput;

            SmartDashboard.putNumber("error total", turnToHubController.getAccumulatedError());

            SmartDashboard.putNumber("error deriv", turnToHubController.getErrorDerivative());

            SmartDashboard.putNumber("error", turnToHubController.getError());

            SmartDashboard.putNumber("feedforward", -feedForward);

            SmartDashboard.putNumber("theta", theta);

            SmartDashboard.putNumber("setpoint angle from turn controller", turnToHubController.getSetpoint());

            SmartDashboard.putNumber("pid output", Math.toRadians(pidOutput));

            SmartDashboard.putNumber("angular speeds that are being fed into robot", speeds.omegaRadiansPerSecond);

            drive(speeds, true, true);


        }));
    
    }

     public boolean isOnBumpWithAngle() {
        double pitch = Math.abs(pigeon.getPitch().getValueAsDouble());
        double roll = Math.abs(pigeon.getRoll().getValueAsDouble());

        //System.out.println("pitch = " + pitch);
        //System.out.println("roll = " + roll);

        return pitch > BUMP_THRESHOLD || roll > BUMP_THRESHOLD;
    }

    public boolean isOnBumpWithLocation() {
       
        Pose2d robotPose = getSavedPose();
        double x = robotPose.getX();
        double y = robotPose.getY();
        
        if (( x > blueLowerX && x < blueUpperX && y > blueLowerY && y < blueUpperY ) 
        || (x > redLowerX && x < redUpperX && y > redLowerY && y < redUpperY)) {
            return true;
        } else {
            return false;
        }

    }

    public double calculateBestAngle() {
        
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
        
        return bestAngleInDegrees;
    }

    public Command turnToBestAngle(Supplier<ChassisSpeeds> speedSupplier){
        return 
        runOnce(
            ()-> {
                turnToAngleController.reset();
            }
        ).andThen(
        run(()->{

            ChassisSpeeds speeds = speedSupplier.get();

            double pidOutput = turnToAngleController.calculate(getSavedPose().getRotation().getDegrees(), calculateBestAngle());

            speeds.omegaRadiansPerSecond = getAngularComponentFromRotationOverride(calculateBestAngle()) + pidOutput;
            
            drive(speeds, true, true);
        }));

    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putData("Field", m_field);

         m_field.getObject("robot pose").setPose(getSavedPose());
        SmartDashboard.putString("robot pose", getSavedPose().toString());

        SmartDashboard.putNumber("robot rotation", getSavedPose().getRotation().getDegrees());

        SmartDashboard.putNumber("best angle", calculateBestAngle());
    }
}