package frc.BisonLib.BaseProject.Swerve;

import static edu.wpi.first.units.Units.Volts;

import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.BisonLib.BaseProject.LimelightHelpers;
import frc.BisonLib.BaseProject.Swerve.Modules.TalonFXModule;
import frc.robot.Constants;

public class SwerveBase extends SubsystemBase {

    protected TalonFXModule[] modules;

    protected final Field2d m_field = new Field2d();
    private final SwerveDrivePoseEstimator odometry;

    // NEVER DIRECTLY CALL ANY GYRO METHODS, ALWAYS USE THE SYNCHRONIZED GYRO LOCK!!
    //private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI, Constants.Swerve.ODOMETRY_UPDATE_RATE_HZ_INTEGER);
    //private final Pigeon2 pigeon;
    //private final BaseStatusSignal yawSignal;
    //private double gyroAccumYawOffset = 0;

    // private final LinearFilter xAccelFilter = LinearFilter.movingAverage(5);
    // private final LinearFilter yAccelFilter = LinearFilter.movingAverage(5);
    private final PIDController thetaController = new PIDController(Constants.Swerve.ROBOT_ROTATION_KP, 0, 0);
    private final BaseStatusSignal[] allOdomSignals;

    protected double max_accel = 0;
    protected double robotRotationError = 0;

    // used for wheel characterization
    protected double initialGyroAngle = 0;
    protected double[] initialPositions = new double[4];

    // odometry stuff
    protected double lastTime = Timer.getFPGATimestamp();
    private LinearFilter lowpass = LinearFilter.movingAverage(50);
    protected double currentTime = 0;
    protected double totalLoopTime = 0;
    protected double inc = 0;
    protected double avgLoopTIme = 0;
    protected double failedOdometryUpdates = 0;
    protected double successfulOdometryUpdates = 0;
    protected double limelightUpdateCounter = 0;


    public final Trigger atRotationSetpoint = new Trigger(()-> Math.abs(robotRotationError) < 1);
    public final Trigger almostAtRotationSetpoint = new Trigger(()-> Math.abs(robotRotationError) < 20);



    // this is a lock to make sure nobody acesses our pose while odometry is updating it
    private final ReentrantReadWriteLock odometryLock = new ReentrantReadWriteLock();

    // this is a lock to make sure nobody acesses our gyro while odometry is updating it
    private final Object gyroLock = new Object();

    private Pose2d currentRobotPose = new Pose2d();
    private SwerveModulePosition[] currentModulePositions = new SwerveModulePosition[4];
    private SwerveModuleState[] currentModuleStates = new SwerveModuleState[4];

    protected String[] camNames;
 
    public SlewRateLimiter omegaFilter = new SlewRateLimiter(Math.toRadians(1074.5588535));
    public SlewRateLimiter xFilter = new SlewRateLimiter(Constants.Swerve.MAX_ACCELERATION_METERS_PER_SECOND_SQ);
    public SlewRateLimiter yFilter = new SlewRateLimiter(Constants.Swerve.MAX_ACCELERATION_METERS_PER_SECOND_SQ);
    //private Pigeon2 pigeon = new Pigeon2(8);

    private VoltageOut m_voltReq;
    //6-11
    // 17 -22
    public int[] validTagIDs;

    public double prevAccelX = 0;
    public double prevAccelY = 0; 


    // SysID
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, // Default ramp rate (1V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
            null, // Default timeout (10s)

            state -> SignalLogger.writeString("State", state.toString())
        ), 
        new SysIdRoutine.Mechanism(
            volts -> {
                modules[0].getTurnMotor().setControl(m_voltReq.withOutput(volts.in(Volts)));
                modules[1].getTurnMotor().setControl(m_voltReq.withOutput(volts.in(Volts)));
                modules[2].getTurnMotor().setControl(m_voltReq.withOutput(volts.in(Volts)));
                modules[3].getTurnMotor().setControl(m_voltReq.withOutput(volts.in(Volts)));
            },
            null, // Left null when using a signal logger
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineSteer; //m_sysIdRoutineSteer; m_sysIdRoutineTranslation

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }
     
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    // 50 Hz Networktables
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable SwerveBaseTable = inst.getTable("SwerveBase");
    private final NetworkTable poseTable = SwerveBaseTable.getSubTable("Pose");
    private final NetworkTable cmdSpeedTable = SwerveBaseTable.getSubTable("Commanded Speed");
    private final NetworkTable odometryTable = SwerveBaseTable.getSubTable("Odometry");
    private final NetworkTable NavXTable = SwerveBaseTable.getSubTable("NavX");
    private final NetworkTable wheelChar = SwerveBaseTable.getSubTable("Wheel Characterization");
    private final NetworkTable poseVisionTable = SwerveBaseTable.getSubTable("Wheel Characterization");

    /**
     * Does all da constructing
     * 
     * @param cameras An array of cameras used for pose estinmation
     * @param moduleTypes The type of swerve module on the swerve drive
     * @param validTagIDs April Tag IDs which are safe to use for pose estimation (stable tags that don't move around too much)
     */
    public SwerveBase(String[] camNames, TalonFXModule[] modules, int[] validTagIDs) {
        //pigeon = new Pigeon2(8, "drivetrain");
        //yawSignal = pigeon.getAccumGyroZ();
        //gyroAccumYawOffset = -yawSignal.getValueAsDouble();
        //pigeon.setYaw(0);
        // 4 modules * 3 signals per module + 1 for pigeon
        allOdomSignals = new BaseStatusSignal[(4 * 3)];
        for(int i = 0; i < modules.length; ++i){
            var signals = modules[i].getOdometrySignals();
            allOdomSignals[i*3 + 0] = signals[0]; // drive position
            allOdomSignals[i*3 + 1] = signals[1]; // drive velocity
            allOdomSignals[i*3 + 2] = signals[2]; // module rotation (cancoder)
        }
        //allOdomSignals[allOdomSignals.length-1] = yawSignal;

        this.camNames = camNames;

        // Holds all the modules
        this.modules = modules;

        this.validTagIDs = validTagIDs;



        /*
        * Sets the gyro at the beginning of the match and 
        * also sets each module state to present
        */

        //maybe have this keep trying to reset the gyro if it fails once
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                setGyro(0);
            } catch (Exception e) {
                setGyro(0);
            }
        }).start();


        thetaController.enableContinuousInput(-180, 180);


        odometryLock.writeLock().lock();
        Rotation2d gyroHeading = getGyroHeading();
        currentModulePositions = getModulePositions();
        try{
            odometry = new SwerveDrivePoseEstimator
            (
                Constants.Swerve.kDriveKinematics, 
                gyroHeading,
                currentModulePositions,
                new Pose2d()
            );
        }finally{
            odometryLock.writeLock().unlock();
        }


        SmartDashboard.putData("field", m_field);
        SmartDashboard.putData("Robot angle PID controller", thetaController);

        m_voltReq = new VoltageOut(0.0); 
    }


    public void playSong(){
        if(modules[0].getModuleType().equals("TalonFXModule")){
            new Thread(() -> {
                try {
                        Orchestra orchestra = new Orchestra();
                        for(var module : this.modules){
                            orchestra.addInstrument(module.getDriveMotor());
                            orchestra.addInstrument(module.getTurnMotor());
                        }

                        var status  = orchestra.loadMusic("EmpireStrikesBack.chrp");
                        if(status.isOK()){
                            double startTime = Timer.getFPGATimestamp();
                            orchestra.play();
                            while((Timer.getFPGATimestamp() - startTime) < 10){}
                            orchestra.stop();
                        }
                        orchestra.close();
                } catch (Exception e) {
                }
            }).start();
        }
    }


    public void pathplannerDriveRobotRelative(ChassisSpeeds speeds){
        driveRobotRelative(speeds, false);
    }


    /*
     * isRedAlliance returns true if we are red alliance and returns false if we are blue alliance
     */
    public boolean isRedAlliance(){
        var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                boolean temp = (alliance.get() == DriverStation.Alliance.Red) ? true : false;
                //SmartDasboard.putBoolean("Alliance", temp);
                return temp;
              }
        //SmartDasboard.putBoolean("Alliance", false);
        return false;
    }


    /*
     * Sets all the swerve modules to the states we want them to be in (velocity + angle)
     */
    public void setModules(SwerveModuleState[] desiredStates, boolean useMaxSpeed) {
        if(useMaxSpeed) SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS_TELEOP);
        else SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND);

        for(var module : modules){
            //SmartDashboard.putString("Swerve/Module State " + module.index, desiredStates[module.index].toString());
            module.setDesiredState(desiredStates[module.index]);
        }
        
    }


    /**
     * setGyro sets the gyro to a given angle,
     * @param degrees the degree value that the gyro should be set to
     */
    public void setGyro(double degrees){
        //pigeon.setYaw(0);
        synchronized(gyroLock){
            // gyro.reset();
            // gyro.setAngleAdjustment(-degrees);
            //gyroAccumYawOffset = -allOdomSignals[allOdomSignals.length-1].getValueAsDouble()/Constants.Swerve.GYRO_DRIFT_COMPENSATION + degrees;
        }
    }


    /**
     * Finds a new angular speed based on rotation override
     * 
     * @param originalSpeeds The original chassis speeds of the robot as inputted by the driver
     * 
     * @return The new rotation component as calculated by the rotation override
     */
    public double getAngularComponentFromRotationOverride(double wantedAngle){
        double currentRotation = getSavedPose().getRotation().getDegrees();
        double pidOutput = thetaController.calculate(currentRotation, wantedAngle);

        robotRotationError = thetaController.getError();
        return MathUtil.clamp(pidOutput, -1, 1) * Constants.Swerve.MAX_ANGULAR_SPEED_RAD_PER_SECOND;
    }


    /**
     * This method should not be touched by anything except odometry, 
     * all angles should be pulled from odometry and not directly from the gyro.
     * This is done to ensure that there is no thread contention for this method.
     * 
     * @returns the angle the gyro is facing expressed as a Rotation2d
     */
    private Rotation2d getGyroHeading() {
        //0.99622314806
        synchronized (gyroLock){
            // double unmoddedGyoHeading = gyroAccumYawOffset + (allOdomSignals[allOdomSignals.length-1].getValueAsDouble()/Constants.Swerve.GYRO_DRIFT_COMPENSATION);
            // SmartDashboard.putNumber("Unmodded Gyro Heading", unmoddedGyoHeading);
            // return Rotation2d.fromDegrees(Math.IEEEremainder(unmoddedGyoHeading, 360));
            //return new Rotation2d(-Math.toRadians(Math.IEEEremainder(gyro.getAngle()/Constants.Swerve.GYRO_DRIFT_COMPENSATION, 360)));

            return new Rotation2d(0);
        }
    }


    /**
     * calculates the distance from the current robot pose to the supplied translation,
     * can be potentially used for figuring out if the robot is in some specific zone
     * 
     * @param other The translation to find the distance to
     * @return The distance from the robot pose to the other supplied translation
     */
    public double getDistanceToTranslation(Translation2d other){
        return getSavedPose().getTranslation().getDistance(other);
    }


    /*
     * Returns the current chassis speeds of the robot, 
     * used with pathplanner
     */
    public ChassisSpeeds getLatestChassisSpeed(){
        ChassisSpeeds speeds;
        odometryLock.readLock().lock();
        try{
            speeds = Constants.Swerve.kDriveKinematics.toChassisSpeeds(currentModuleStates);
        }finally{
            odometryLock.readLock().unlock();
        }
        return speeds;
    }


    public SwerveModuleState[] getLatestModuleStates(){
        SwerveModuleState[] states;
        odometryLock.readLock().lock();
        try{
            states = currentModuleStates;
        }finally{
            odometryLock.readLock().unlock();
        }
        return states;
    }


    /**
     * @returns an array containing the position of each swerve module (check SwerveModule.java for further details)
     */
    public SwerveModulePosition[] getModulePositions() {

        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

        for(var module : modules){
            modulePositions[module.index] = module.getPosition();
        }

        return modulePositions;
    }


    /**
     * @return an array containing the state (velocity + rotation) of each swerve module
     */
    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];

        for(var module : modules){
            states[module.index] = module.getState();
        }

        return states;
    }


    /**
     * Gets the acceleration of each swerve module, 
     * keep in mind these can be slightly inaccurate because of wheel slippage
     * 
     * @return a double array that contains the acceleration of each swerve module
     */
    public double[] getModuleAccelerations(){
        double[] accelerations = new double[4];

        for(var module : modules){
            accelerations[module.index] = module.getDriveAcceleration();
        }

        return accelerations;
    }


    public double[] getRawDrivePositions(){
        double[] positions = new double[4];

        for(var module : modules) {
            positions[module.index] = module.getUncachedDrivePosition();
        }

        return positions;
    }


    /**
     * Manually resets the odometry to a given pose
     * Also resets the gyro
     * Pretty much only used at the start of auton
     * 
     * @param pose The pose to set the robot pose to
     */
    public void resetOdometry(Pose2d pose) {
        setGyro(pose.getRotation().getDegrees());
        Rotation2d gyroHeading = getGyroHeading();
        odometryLock.writeLock().lock();
        try{
            odometry.resetPosition(
                gyroHeading,
                currentModulePositions,
                pose);
        }finally{
            odometryLock.writeLock().unlock();
        }
    }


    /**
     * resetGyro sets the gyro to "facing away from the driver station"
     * 
     * @return A command that "zeroes" our gyro
     */
    public Command resetGyro() {
        return resetGyro(180);
    }

    
    public Command backwardsResetGyro(){
        return resetGyro(0);
    }


    public Command driveAtSpeed (double speed) {
        return run (() -> {

            ChassisSpeeds speeds =
                    new ChassisSpeeds(
                        MathUtil.clamp(speed, -Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND, Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND),
                        MathUtil.clamp(speed, -Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND, Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND),
                    getAngularComponentFromRotationOverride(0)
                );
                SmartDashboard.putString("align speeds", speeds.toString());

                drive(speeds, true, false);
        });
    }

    public Command resetGyro(double angle){
        return runOnce(
            ()-> 
                {
                    if(isRedAlliance()){
                        odometryLock.writeLock().lock();
                        try{
                            resetOdometry(new Pose2d(currentRobotPose.getX(), currentRobotPose.getY(), Rotation2d.fromDegrees(angle)));
                        }finally{
                            odometryLock.writeLock().unlock();
                        }
                    }
                    else {
                        odometryLock.writeLock().lock();
                        try{
                            resetOdometry(new Pose2d(currentRobotPose.getX(), currentRobotPose.getY(), Rotation2d.fromDegrees(angle+180)));
                        }finally{
                            odometryLock.writeLock().unlock();
                        }
                    }
                }
        ).ignoringDisable(true).andThen(cameraSeedCommand());
    }

    public Command cameraSeedCommand(){
        return runOnce(()-> {
            seedCameraHeading();
        }).ignoringDisable(true);
    }

    public void seedCameraHeading(){
        //LL RESET
        for(String cam: camNames){
            LimelightHelpers.SetIMUMode(cam, 1);
            LimelightHelpers.SetRobotOrientation(cam, getSavedPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
            LimelightHelpers.SetIMUMode(cam, 1);
        }
    }

    /*
     * Calculates the circumference of the wheel by turning in place slowly
     * 
     * COMMENT OUT DRIVEBASE AND ODOMETRY CODE BEFORE RUNNING THIS
     */
    // public Command runWheelCharacterization() {

    //     /*
    //      * wheel Base is the width or distance from one wheel to the next on the chassis
    //      * let wheel base = w
    //      * sqrt((w / 2)^2+(w/2)^2) = distance from wheel to center, or radius = sqrt2 *
    //      * w/2
    //      * multiply by 2 to get diameter --> d = sqrt2 * w
    //      * multiply by pi to get circumference:
    //      */

    //     double oneRotation = Constants.Swerve.WHEEL_BASE_METERS * Math.PI * Math.sqrt(2);
    //     backwardsResetGyro();
    //     initialGyroAngle = gyro.getAngle();
        
    //     return runOnce(() -> {
            
    //         // get each module in positions
    //         // sets each motor to stop moving, and converts the module index (which quadrant
    //         // relative to the chassis the motor is - 1) to degrees
    //         // for (var mod : modules) {
    //         //     mod.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45 + mod.index * 90)));
    //         // }
            
    //     })
    //     .andThen(waitSeconds(0.5))
    //     .andThen(
    //         deadline(
    //             new WaitCommand(6),
    //             run(() -> {
    //             drive( new ChassisSpeeds(0.0,0.0,0.3),false,false);
    //     }))).andThen(runOnce(() -> {
    //         //stop motors
    //         for (var mod : modules) {
    //             mod.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45 + mod.index * 90)));
    //         }

    //     })).andThen(waitSeconds(1))
    //     .andThen(runOnce(() -> {

    //             double[] currentPositions = getRawDrivePositions();
    //             double avg_calculated_wheel_circumference = 0;
    //             //distance of one rotation * number of rotations based on gyro
    //             double actual_distance_traveled =  oneRotation * Math.abs(gyro.getAngle()/Constants.Swerve.GYRO_DRIFT_COMPENSATION) / 360;
    //             for (var mod : modules) {

    //                 //actual distance / wheel rotations = wheel circumference, because wheel circumference * number of rotations = linear distance the wheel travels
    //                 //actual distance / (pi * wheel rotations (current rotations - original rotations) / gear ratio to account for motor spins per wheel spin)
    //                 double new_circumference = actual_distance_traveled / 
    //                 (Math.PI * (currentPositions[mod.index] - initialPositions[mod.index]) /((Constants.Swerve.DRIVING_GEAR_RATIO)));
                    
    //                 avg_calculated_wheel_circumference += Math.abs(new_circumference);

    //                 SmartDashboard.putNumber("Actual Distance", actual_distance_traveled);
    //                 SmartDashboard.putNumber("new circumference " + mod.index, new_circumference);
    //                 SmartDashboard.putNumber("raw initial distance " + mod.index, initialPositions[mod.index]);
    //                 SmartDashboard.putNumber("raw current distance " + mod.index, currentPositions[mod.index]);
    //                 SmartDashboard.putNumber(mod.index + "calculated wheel circumference", new_circumference);
    //             }
    //             avg_calculated_wheel_circumference /= 4;
    //             SmartDashboard.putNumber("Average Calculated Wheel Circumference", avg_calculated_wheel_circumference);
    //         }));
    // }


    public Command requireSubsystem(){
        return new WaitCommand(0);
    }
    

    /*
     * Stops all of the swerve modules
     */
    public void stopModules() {
        for(var module : modules){
            module.stop();
        }
    }


    /**
     * Continuously rotates robot to the specified angle while maintaining normal driver control of the robot
     * 
     * @param angleDegrees The angle in degrees that the robot should turn to, 
     *                     this is a double supplier so you can continously pass different values
     * 
     * @return Returns a functional command that will rotate the robot to a specified angle, 
     *         when interrupted, will return driver control to robot rotation
     */
    public Command rotateToAngle(DoubleSupplier angleDegrees, Supplier<ChassisSpeeds> speedSupplier){
        return run
        (
            /* EXCECUTE */
            ()-> {
                    ChassisSpeeds speeds = speedSupplier.get();
                    speeds.omegaRadiansPerSecond = getAngularComponentFromRotationOverride(angleDegrees.getAsDouble());
                    drive(speeds, true, true);
                 }
        );
    }

    /**
     * Drives swerve given chassis speeds robot relative
     * DOESN'T APPLY ACCELERATION LIMITS
     * 
     * @param chassisSpeeds The chassis speeds the robot should travel at
     */
    protected void driveRobotRelative(ChassisSpeeds chassisSpeeds, boolean useMaxSpeed) {
        var tmpStates = Constants.Swerve.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        if(useMaxSpeed) SwerveDriveKinematics.desaturateWheelSpeeds(tmpStates, Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS_TELEOP);
        else SwerveDriveKinematics.desaturateWheelSpeeds(tmpStates, Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND);
        var speeds = Constants.Swerve.kDriveKinematics.toChassisSpeeds(tmpStates);

        Rotation2d skewCompensationFactor = Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * Constants.Swerve.SKEW_COMPENSATION_RATE);

        chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getSavedPose().getRotation()),
            getSavedPose().getRotation().plus(skewCompensationFactor));

        //SmartDashboard.putString("Swerve/Commanded Chassis Speeds", chassisSpeeds.toString());
        // convert chassis speeds to module states
        SwerveModuleState[] moduleStates = Constants.Swerve.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // set the modules to their desired speeds
        setModules(moduleStates, useMaxSpeed);
        
    }

    /*
     * Drives the robot in teleop, we don't want it fighting the auton swerve commands
     */
    public void teleopDefaultCommand(Supplier<ChassisSpeeds> speedsSupplier, boolean fieldOriented){
        drive(speedsSupplier.get(), true, true);
    } //590, 736
    

    //CommandedSpeeds
    private final DoublePublisher omega = cmdSpeedTable.getDoubleTopic("Zj").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher vx = cmdSpeedTable.getDoubleTopic("Xj").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher vy = cmdSpeedTable.getDoubleTopic("Yj").publish(PubSubOption.periodic(0.02));
    /**
     * Drives swerve given chassis speeds
     * Should be called every loop
     * 
     * @param commandedSpeeds the commanded chassis speeds from the joysticks
     * @param fieldOriented A boolean that specifies if the robot should be driven in fieldOriented mode or not
     */
    public void drive(ChassisSpeeds commandedSpeeds, boolean fieldOriented, boolean useMaxSpeed){

        ChassisSpeeds currentFieldRelSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(getLatestChassisSpeed(), getSavedPose().getRotation());

        // I made the convention "v" means robots actual vel and "w" is robots wanted vel
        double v_x = currentFieldRelSpeeds.vxMetersPerSecond;
        double v_y = currentFieldRelSpeeds.vyMetersPerSecond;
        double w_x = commandedSpeeds.vxMetersPerSecond;
        double w_y = commandedSpeeds.vyMetersPerSecond;
        double dt = 0.02;
        
        double v_mag = Math.hypot(v_x, v_y);
        double w_along_v = 0.0;
        
        if (v_mag > 1e-6) {
            // project w onto v (commanded velocity component along current velocity direction)
            double dot = v_x * w_x + v_y * w_y; // w · v
            w_along_v = dot / v_mag;            // signed scalar projection
        } else {
            // if robot is nearly stopped, just use commanded speed magnitude (all commanded vel is speeding us up)
            w_along_v = Math.hypot(w_x, w_y);
        }

        // the signum signifies if we are requesting speed up/braking
        double max_fwd_accel = Constants.Swerve.MAX_ACCELERATION_METERS_PER_SECOND_SQ *
                               (1 - Math.signum(w_along_v - v_mag) * v_mag / Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS_TELEOP);

        if(w_along_v - v_mag > 0){
            max_fwd_accel = Constants.Swerve.MAX_ACCELERATION_METERS_PER_SECOND_SQ *
                               (1 -  (v_mag / Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS_TELEOP));
        }
        else{
            max_fwd_accel = Constants.Swerve.MAX_ACCELERATION_METERS_PER_SECOND_SQ *
                               1.5 * v_mag / Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS_TELEOP;
        }
        
        double desiredForwardAccel = (w_along_v - v_mag)/dt;

        // project commanded vel onto forward axis, if we aren't moving rn then all wanted vel is parallel
        double w_parallel_x = (v_mag > 1e-6) ? (v_x / v_mag) * w_along_v : w_x;
        double w_parallel_y = (v_mag > 1e-6) ? (v_y / v_mag) * w_along_v : w_y;

        // perpendicular component = commanded - parallel
        double w_perp_x = w_x - w_parallel_x;
        double w_perp_y = w_y - w_parallel_y;
        double w_perp_mag = Math.hypot(w_perp_x, w_perp_y);

        // current sideways vel is always 0 since no component of the current vel doesn't point in the direction of the current vel
        double desiredSkidAccel = w_perp_mag/dt;

        // make sure total accel doesnt exceed max accel
        double norm = Math.pow(desiredForwardAccel/max_fwd_accel, 2)
                    + Math.pow(desiredSkidAccel/Constants.Swerve.MAX_SKID_ACCEL, 2);
        if(norm > 1){
            double scale = 1 / Math.sqrt(norm);
            desiredForwardAccel *= scale;
            desiredSkidAccel *= scale;
        }
        else{
        }
        double newForwardVel = v_mag + desiredForwardAccel * dt;

        double vx_forward;
        double vy_forward;
        if (v_mag > 1e-6) {
            vx_forward = (v_x/v_mag) * newForwardVel;
            vy_forward = (v_y/v_mag) * newForwardVel;
        }
        else{
            // if stopped, use commanded direction for initial forward push (preserves user intent)
            double w_mag = Math.hypot(w_x, w_y);
            if (w_mag > 1e-6) {
                vx_forward = (w_x / w_mag) * newForwardVel;
                vy_forward = (w_y / w_mag) * newForwardVel;
            } else {
                vx_forward = 0.0;
                vy_forward = 0.0;
            }
        }
        
        double vx_perp = 0.0;
        double vy_perp = 0.0;
        if(w_perp_mag > 1e-6) {
            vx_perp = (w_perp_x / w_perp_mag) * desiredSkidAccel * dt;
            vy_perp = (w_perp_y / w_perp_mag) * desiredSkidAccel * dt;
        }

        // vx_perp = 0;
        // vx_perp = 0;
        commandedSpeeds.vxMetersPerSecond = vx_forward + vx_perp;
        commandedSpeeds.vyMetersPerSecond = vy_forward + vy_perp;
        //commandedSpeeds.vxMetersPerSecond = xFilter.calculate(commandedSpeeds.vxMetersPerSecond);
        //commandedSpeeds.vyMetersPerSecond = yFilter.calculate(commandedSpeeds.vyMetersPerSecond);
        commandedSpeeds.omegaRadiansPerSecond = omegaFilter.calculate(commandedSpeeds.omegaRadiansPerSecond);
        //speeds = applyAccelerationLimit(speeds);

        omega.set(commandedSpeeds.omegaRadiansPerSecond);
        vx.set(commandedSpeeds.vxMetersPerSecond);
        vy.set(commandedSpeeds.vyMetersPerSecond);

        this.driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(commandedSpeeds, getSavedPose().getRotation()), useMaxSpeed);

        //SmartDashboard.putBoolean("collision", detectCollision());

    }


    public void updateOdometryWithKinematics(){
        lastTime = currentTime;
        currentTime = Timer.getFPGATimestamp();
        totalLoopTime += (currentTime-lastTime);
        avgLoopTIme = lowpass.calculate(currentTime - lastTime);

        StatusCode signal = BaseStatusSignal.waitForAll(2 / Constants.Swerve.ODOMETRY_UPDATE_RATE_HZ_INTEGER, allOdomSignals);
        if(signal.isError()){
            ++failedOdometryUpdates;
        }
        else{
            ++successfulOdometryUpdates;
        }

        SwerveModulePosition[] positions = getModulePositions();
        SwerveModuleState[] states = getModuleStates();
        odometryLock.writeLock().lock();
        try{
            currentModulePositions = positions;
            currentRobotPose = odometry.updateWithTime(
                Timer.getFPGATimestamp(),
                getGyroHeading(),
                currentModulePositions);
            currentModuleStates = states;
        }finally{
            odometryLock.writeLock().unlock();
        }
    }

    public DoublePublisher avgTagDistance = null;
    public StringPublisher estimatePose = null;

    /*
     * updateOdometryWithVision uses vision to add measurements to the odometry
     */
    public void updateOdometryWithVision(boolean reSeedGyro){
        if(reSeedGyro){
            seedCameraHeading();
        }
        int inc = 0;
        double avgLLx = 0;
        double avgLLy = 0;
        double avgLLomega = 0;
        for(String cam : camNames){  
            LimelightHelpers.SetRobotOrientation(cam, getSavedPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
            LimelightHelpers.SetFiducialIDFiltersOverride(cam, validTagIDs);
            LimelightHelpers.PoseEstimate mt2_estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cam);
        
            // Only update pose if it is valid and if we arent spinning too fast
            if(mt2_estimate != null && mt2_estimate.tagCount != 0){//remove rotation speed limit
                ++inc;
                avgTagDistance = poseVisionTable.getDoubleTopic(inc + " Average Tag Distance").publish(PubSubOption.periodic(0.02));
                avgTagDistance.set(mt2_estimate.avgTagDist);
                avgLLx += mt2_estimate.pose.getX();
                avgLLy += mt2_estimate.pose.getY();
                avgLLomega += mt2_estimate.pose.getRotation().getDegrees();
                // Finally, we actually add the measurement to our odometry
                odometryLock.writeLock().lock();
                try{
                    odometry.addVisionMeasurement
                    (
                        mt2_estimate.pose, 
                        mt2_estimate.timestampSeconds,
                        
                        // This way it doesn't trust the rotation reading from the vision
                        // these are all the state stdevs
                        VecBuilder.fill(mt2_estimate.avgTagDist * 0.1/4.3, mt2_estimate.avgTagDist * 0.1/4.3, 999999999)
                    );
                }finally{
                    odometryLock.writeLock().unlock();
                }
                
                // This puts the pose reading from each camera onto the Field2d Widget,
                // Docs - https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/field2d-widget.html
                m_field.getObject(cam).setPose(mt2_estimate.pose);
                estimatePose = poseVisionTable.getStringTopic("mt2 pose").publish(PubSubOption.periodic(0.02));
                estimatePose.set(mt2_estimate.pose.toString());
            }
            avgLLx /= inc;
            avgLLy /= inc;
            avgLLomega /= inc;

            SmartDashboard.putString("Logged Pose", new Pose2d(avgLLx, avgLLy, Rotation2d.fromDegrees(avgLLomega)).toString());
        }  
    }


    /**
     * Used to find the max stator current to prevent wheel slip
     * https://pro.docs.ctr-electronics.com/en/latest/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
     * 
     * 
     * @param voltageSupplier
     */
    public void testModules(double voltage){
        //SmartDasboard.putNumber("Swerve/Module 1/Module 1 Current", modules[0].getDriveStatorCurrent());
        //SmartDashboard.putNumber("Swerve/Module 2/Module 2 Current", modules[1].getDriveStatorCurrent());
        //SmartDashboard.putNumber("Swerve/Module 3/Module 3 Current", modules[2].getDriveStatorCurrent());
        //SmartDashboard.putNumber("Swerve/Module 4/Module 4 Current", modules[3].getDriveStatorCurrent());

        for(var mod : modules){
            mod.driveWithVoltage(voltage);
        }
    }

    public Pose2d getSavedPose(){
        Pose2d pose;
        odometryLock.readLock().lock();
        try{
            pose = currentRobotPose;
        }finally{
            odometryLock.readLock().unlock();
        }
        return pose;
    }

    //Odometry
    private final DoublePublisher AvgOdometryLoopTime = odometryTable.getDoubleTopic("Average odometry loop time").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher FailedOdometryUpdates = odometryTable.getDoubleTopic("Failed odometry updates").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher SuccessfulOdometryUpdates = odometryTable.getDoubleTopic("Sucessful odometry updates").publish(PubSubOption.periodic(0.02));

    //NavX
    private final DoublePublisher NavXPos = NavXTable.getDoubleTopic("NavX Position").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher NavXTemp = NavXTable.getDoubleTopic("NavX temperature").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher NavXModPos = NavXTable.getDoubleTopic("NavX Modified Position").publish(PubSubOption.periodic(0.02));

    //Pose
    private final StringPublisher robotPose = poseTable.getStringTopic("robot pose").publish(PubSubOption.periodic(0.02));
    private final BooleanPublisher atRotSetpoint = poseTable.getBooleanTopic("at rotation setpoint").publish(PubSubOption.periodic(0.02));


    @Override
    public void periodic() {
        AvgOdometryLoopTime.set(avgLoopTIme);
        FailedOdometryUpdates.set(failedOdometryUpdates);
        SuccessfulOdometryUpdates.set(successfulOdometryUpdates);
        robotPose.set(getSavedPose().toString());

        // limelightUpdateCounter++;
        // if(limelightUpdateCounter > 25){
        //     updateOdometryWithVision(true);
        //     limelightUpdateCounter = 0;
        // }
        // else{
            updateOdometryWithVision(false);
        //}

       //SmartDashboard.putNumber("Pigeon Yaw", pigeon.getYaw().getValueAsDouble());
        //NavXPos.set(gyro.getAngle());
        //NavXTemp.set(gyro.getTempC());
        NavXModPos.set(getGyroHeading().getDegrees());

        m_field.setRobotPose(getSavedPose());

        SwerveModuleState[] modStates = getModuleStates();

        SmartDashboard.putNumber("Module 1 Angle deg", modStates[0].angle.getDegrees());
        SmartDashboard.putNumber("Module 2 Angle deg", modStates[1].angle.getDegrees());
        SmartDashboard.putNumber("Module 3 Angle deg", modStates[2].angle.getDegrees());
        SmartDashboard.putNumber("Module 4 Angle deg", modStates[3].angle.getDegrees());        
        
        SmartDashboard.putBoolean("Robot Rotation at Setpoint", atRotationSetpoint.getAsBoolean());

        if (currentModuleStates[0] != null) {
            ChassisSpeeds currentFieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(getLatestChassisSpeed(), getSavedPose().getRotation());
            double currentvx = currentFieldRelativeSpeeds.vxMetersPerSecond;
            double currentvy = currentFieldRelativeSpeeds.vyMetersPerSecond;
            SmartDashboard.putNumber("Currentvx", currentvx);
            SmartDashboard.putNumber("Currentvy", currentvy);
        }
    }
}