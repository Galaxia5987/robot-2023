package frc.robot.autonomous;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gyroscope.Gyroscope;

import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

/**
 * Custom PathPlanner version of SwerveControllerCommand
 */
public class FollowPath extends CommandBase {
    //    private static final FollowPathLogAutoLogged log = new FollowPathLogAutoLogged();
    private static Consumer<PathPlannerTrajectory> logActiveTrajectory = null;
    private static Consumer<Pose2d> logTargetPose = null;
    private static Consumer<ChassisSpeeds> logSetpoint = null;
    private static BiConsumer<Translation2d, Rotation2d> logError =
            FollowPath::defaultLogError;
    private final Timer timer = new Timer();
    private final Supplier<PathPlannerTrajectory> trajectory;
    private final Supplier<Pose2d> poseSupplier;
    private final SwerveDriveKinematics kinematics;
    private final PPHolonomicDriveController controller;
    private final Consumer<SwerveModuleState[]> outputModuleStates;
    private final Consumer<ChassisSpeeds> outputChassisSpeeds;
    private final boolean useKinematics;
    private PathPlannerTrajectory transformedTrajectory;

    /**
     * Constructs a new PPSwerveControllerCommand that when executed will follow the provided
     * trajectory. This command will not return output voltages but rather raw module states from the
     * position controllers which need to be put into a velocity PID.
     *
     * <p>Note: The controllers will *not* set the output to zero upon completion of the path- this is
     * left to the user, since it is not appropriate for paths with nonstationary endstates.
     *
     * @param trajectory         The trajectory to follow.
     * @param poseSupplier       A function that supplies the robot pose - use one of the odometry classes
     *                           to provide this.
     * @param kinematics         The kinematics for the robot drivetrain.
     * @param xController        The Trajectory Tracker PID controller for the robot's x position.
     * @param yController        The Trajectory Tracker PID controller for the robot's y position.
     * @param rotationController The Trajectory Tracker PID controller for angle for the robot.
     * @param outputModuleStates The raw output module states from the position controllers.
     * @param requirements       The subsystems to require.
     */
    public FollowPath(
            Supplier<PathPlannerTrajectory> trajectory,
            Supplier<Pose2d> poseSupplier,
            SwerveDriveKinematics kinematics,
            PIDController xController,
            PIDController yController,
            PIDController rotationController,
            Consumer<SwerveModuleState[]> outputModuleStates,
            Subsystem... requirements) {
        this.trajectory = trajectory;
        this.poseSupplier = poseSupplier;
        this.kinematics = kinematics;
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        this.controller = new PPHolonomicDriveController(xController, yController, rotationController);
        this.outputModuleStates = outputModuleStates;
        this.outputChassisSpeeds = null;
        this.useKinematics = true;

        addRequirements(requirements);
    }

    private static void defaultLogError(Translation2d translationError, Rotation2d rotationError) {
        SmartDashboard.putNumber("PPSwerveControllerCommand/xErrorMeters", translationError.getX());
        SmartDashboard.putNumber("PPSwerveControllerCommand/yErrorMeters", translationError.getY());
        SmartDashboard.putNumber(
                "PPSwerveControllerCommand/rotationErrorDegrees", rotationError.getDegrees());
    }

    /**
     * Set custom logging callbacks for this command to use instead of the default configuration of
     * pushing values to SmartDashboard
     *
     * @param logActiveTrajectory Consumer that accepts a PathPlannerTrajectory representing the
     *                            active path. This will be called whenever a PPSwerveControllerCommand starts
     * @param logTargetPose       Consumer that accepts a Pose2d representing the target pose while path
     *                            following
     * @param logSetpoint         Consumer that accepts a ChassisSpeeds object representing the setpoint
     *                            speeds
     * @param logError            BiConsumer that accepts a Translation2d and Rotation2d representing the error
     *                            while path following
     */
    public static void setLoggingCallbacks(
            Consumer<PathPlannerTrajectory> logActiveTrajectory,
            Consumer<Pose2d> logTargetPose,
            Consumer<ChassisSpeeds> logSetpoint,
            BiConsumer<Translation2d, Rotation2d> logError) {
        FollowPath.logActiveTrajectory = logActiveTrajectory;
        FollowPath.logTargetPose = logTargetPose;
        FollowPath.logSetpoint = logSetpoint;
        FollowPath.logError = logError;
    }

    public static Command loadTrajectory(String path, Function<PathPlannerTrajectory, Command> resetCommand) {
        SwerveDrive swerveDrive = SwerveDrive.getInstance();
        final Supplier<PathPlannerTrajectory> pathSupplier = () -> PathPlannerTrajectory.transformTrajectoryForAlliance(
                PathPlanner.loadPath(path, SwerveConstants.MAX_VELOCITY_AUTO, SwerveConstants.MAX_ACCELERATION_AUTO),
                DriverStation.getAlliance()
        );
        return new ProxyCommand(() -> resetCommand.apply(pathSupplier.get())).andThen(new FollowPath(
                pathSupplier,
                swerveDrive::getBotPose,
                swerveDrive.getKinematics(),
                new PIDController(SwerveConstants.AUTO_X_Kp, SwerveConstants.AUTO_X_Ki, SwerveConstants.AUTO_X_Kd),
                new PIDController(SwerveConstants.AUTO_Y_Kp, SwerveConstants.AUTO_Y_Ki, SwerveConstants.AUTO_Y_Kd),
                new PIDController(SwerveConstants.AUTO_ROTATION_Kp, SwerveConstants.AUTO_ROTATION_Ki, SwerveConstants.AUTO_ROTATION_Kd),
                swerveDrive::setModuleStates,
                swerveDrive
        ));
    }

    public static Command loadTrajectory(String path) {
        SwerveDrive swerveDrive = SwerveDrive.getInstance();
        final Supplier<PathPlannerTrajectory> pathSupplier = () -> PathPlannerTrajectory.transformTrajectoryForAlliance(
                PathPlanner.loadPath(path, SwerveConstants.MAX_VELOCITY_AUTO, SwerveConstants.MAX_ACCELERATION_AUTO),
                DriverStation.getAlliance()
        );
        return new FollowPath(
                pathSupplier,
                swerveDrive::getBotPose,
                swerveDrive.getKinematics(),
                new PIDController(SwerveConstants.AUTO_X_Kp, SwerveConstants.AUTO_X_Ki, SwerveConstants.AUTO_X_Kd),
                new PIDController(SwerveConstants.AUTO_Y_Kp, SwerveConstants.AUTO_Y_Ki, SwerveConstants.AUTO_Y_Kd),
                new PIDController(SwerveConstants.AUTO_ROTATION_Kp, SwerveConstants.AUTO_ROTATION_Ki, SwerveConstants.AUTO_ROTATION_Kd),
                swerveDrive::setModuleStates,
                swerveDrive
        );
    }

    public static Function<PathPlannerTrajectory, Command> resetCommand(SwerveDrive swerveDrive, Gyroscope gyroscope) {
        return (p) ->
                new InstantCommand(() -> {
                    swerveDrive.resetPose(p.getInitialPose());
                    gyroscope.resetYaw(p.getInitialHolonomicPose().getRotation());
                });
    }

    public static void setAllianceColor(DriverStation.Alliance allianceColor){
//        this.
    }

    @Override
    public void initialize() {
        transformedTrajectory = trajectory.get();

        if (logActiveTrajectory != null) {
            logActiveTrajectory.accept(transformedTrajectory);
        }

        timer.reset();
        timer.start();

        PathPlannerServer.sendActivePath(transformedTrajectory.getStates());
    }

    @Override
    public void execute() {
        double currentTime = this.timer.get();
        PathPlannerState desiredState = (PathPlannerState) transformedTrajectory.sample(currentTime);

        Pose2d currentPose = this.poseSupplier.get();

        PathPlannerServer.sendPathFollowingData(
                new Pose2d(desiredState.poseMeters.getTranslation(), desiredState.holonomicRotation),
                currentPose);

        ChassisSpeeds targetChassisSpeeds = this.controller.calculate(currentPose, desiredState);
        targetChassisSpeeds.vxMetersPerSecond += targetChassisSpeeds.vxMetersPerSecond * SwerveConstants.AUTO_X_Kf;
        targetChassisSpeeds.vyMetersPerSecond += targetChassisSpeeds.vyMetersPerSecond * SwerveConstants.AUTO_Y_Kf;
        targetChassisSpeeds.omegaRadiansPerSecond -= SwerveConstants.AUTO_ROTATION_Kf;

        if (this.useKinematics) {
            SwerveModuleState[] targetModuleStates =
                    this.kinematics.toSwerveModuleStates(targetChassisSpeeds);

            this.outputModuleStates.accept(targetModuleStates);
        } else {
            this.outputChassisSpeeds.accept(targetChassisSpeeds);
        }

        if (logTargetPose != null) {
            logTargetPose.accept(
                    new Pose2d(desiredState.poseMeters.getTranslation(), desiredState.holonomicRotation));
        }

        if (logError != null) {
            logError.accept(
                    currentPose.getTranslation().minus(desiredState.poseMeters.getTranslation()),
                    currentPose.getRotation().minus(desiredState.holonomicRotation));
        }

        if (logSetpoint != null) {
            logSetpoint.accept(targetChassisSpeeds);
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.timer.stop();

        if (interrupted
                || Math.abs(transformedTrajectory.getEndState().velocityMetersPerSecond) < 0.1) {
            if (useKinematics) {
                this.outputModuleStates.accept(
                        this.kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0)));
            } else {
                this.outputChassisSpeeds.accept(new ChassisSpeeds());
            }
        }
    }

    @Override
    public boolean isFinished() {
        return this.timer.hasElapsed(transformedTrajectory.getTotalTimeSeconds())
                || (trajectory == null);
    }
}
