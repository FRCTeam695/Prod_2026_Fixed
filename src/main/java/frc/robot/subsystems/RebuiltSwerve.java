package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;

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
    
    private final ProfiledPIDController turnController = new ProfiledPIDController(0.075, 0, 0, new TrapezoidProfile.Constraints(700, 1000));

    private final PIDController controller = new PIDController(0.06, 0, 0);

    public final Trigger onBump;
    
    protected final Field2d m_field = new Field2d();
    
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
        turnController.enableContinuousInput(-180, 180);

        controller.enableContinuousInput(-180, 180);

        onBump = new Trigger(
            () -> (isOnBumpWithAngle() && isOnBumpWithLocation())
        );

    }


    public Command rotateTowardsVirtualHub(Supplier<ChassisSpeeds> wantedSpeeds, Supplier<Pose2d> hubPose){
        return 
        runOnce(
            ()-> {
                controller.reset();
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

            double pidOutput = controller.calculate(robotPose.getRotation().getDegrees(), theta);

            getAngularComponentFromRotationOverride(theta); // this line is currently just for logging purposes

            speeds.omegaRadiansPerSecond = -feedForward + getAngularComponentFromRotationOverride(theta) + 
            pidOutput;

            SmartDashboard.putNumber("error total", controller.getAccumulatedError());

            SmartDashboard.putNumber("error deriv", controller.getErrorDerivative());

            SmartDashboard.putNumber("error", controller.getError());

            SmartDashboard.putNumber("feedforward", -feedForward);

            SmartDashboard.putNumber("theta", theta);

            SmartDashboard.putNumber("setpoint angle from turn controller", controller.getSetpoint());

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

    public double calculateBumpRotationSetpointDegrees() {
        
        double r = getSavedPose().getRotation().getDegrees() + 180;

        double bestAngle = 0;

        // if between 0 and 90, turnm to 45
        // if between 91 and 180, turn to 135
        // if between 181 and 270, turn to 225
        // if betwenen 271 and 360, turn to 315

        if (r > 0 && r < 90) {
            bestAngle = 45;
        } else if (r > 91 && r < 180) {
            bestAngle = 135;
        } else if (r > 181 && r < 270) {
            bestAngle = 225;
        } else if (r > 271 && r < 360) {
            bestAngle = 315;
        }

        return bestAngle;
    }

    public Command turnOnBump(Supplier<ChassisSpeeds> speedSupplier){
        return new ConditionalCommand(
            rotateToAngle(
                () -> calculateBumpRotationSetpointDegrees(), speedSupplier
            ).until(
                atRotationSetpoint
            ), 
            null, 
            onBump
        );
    }

    @Override
    public void periodic() {
        SmartDashboard.putData("Field", m_field);

         m_field.getObject("robot pose").setPose(getSavedPose());
        SmartDashboard.putString("robot pose", getSavedPose().toString());

        SmartDashboard.putNumber("robot rotation", getSavedPose().getRotation().getDegrees());
    }
}