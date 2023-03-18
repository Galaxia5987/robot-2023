package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.LoggedSubsystem;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.utils.Utils;

import java.util.Arrays;

import static frc.robot.Ports.SwerveDrive.*;
import static frc.robot.subsystems.drivetrain.SwerveConstants.*;

public class SwerveDrive extends LoggedSubsystem<SwerveDriveLogInputs> {
    private static SwerveDrive INSTANCE;
    private final Limelight limelight = Limelight.getInstance();
    private final Gyroscope gyroscope = Gyroscope.getInstance();
    private final SwerveDriveKinematics mKinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Rear left
            new Translation2d(-DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Rear right
            new Translation2d(-DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));
    private final SwerveModule mFrontLeft;
    private final SwerveModule mFrontRight;
    private final SwerveModule mRearLeft;
    private final SwerveModule mRearRight;
    private SwerveModuleState[] swerveModuleStates = new SwerveModuleState[]{
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
    };
    private SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[]{
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
    };

    private final SwerveDriveOdometry mOdometry = new SwerveDriveOdometry(mKinematics, new Rotation2d(),
            swerveModulePositions);

//    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
//            mKinematics,
//            new Rotation2d(),
//            Arrays.copyOf(swerveModulePositions, swerveModulePositions.length),
//            new Pose2d()
//    );

    private SwerveDrive() {
        super(new SwerveDriveLogInputs());

        mFrontLeft = new SwerveModule(
                Module.FL,
                FRONT_LEFT_MODULE_DRIVE_MOTOR_ID,
                FRONT_LEFT_MODULE_STEER_MOTOR_ID,
                6,
                OFFSETS[Module.FL.number],
                FRONT_LEFT_DRIVE_INVERTED,
                FRONT_LEFT_ANGLE_INVERTED,
                FRONT_LEFT_MOTION_MAGIC_CONFIGS);
        mFrontRight = new SwerveModule(
                Module.FR,
                FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID,
                FRONT_RIGHT_MODULE_STEER_MOTOR_ID,
                8,
                OFFSETS[Module.FR.number],
                FRONT_RIGHT_DRIVE_INVERTED,
                FRONT_RIGHT_ANGLE_INVERTED,
                FRONT_RIGHT_MOTION_MAGIC_CONFIGS);
        mRearLeft = new SwerveModule(
                Module.RL,
                REAR_LEFT_MODULE_DRIVE_MOTOR_ID,
                REAR_LEFT_MODULE_STEER_MOTOR_ID,
                5,
                OFFSETS[Module.RL.number],
                REAR_LEFT_DRIVE_INVERTED,
                REAR_LEFT_ANGLE_INVERTED,
                REAR_LEFT_MOTION_MAGIC_CONFIGS);
        mRearRight = new SwerveModule(
                Module.RR,
                REAR_RIGHT_MODULE_DRIVE_MOTOR_ID,
                REAR_RIGHT_MODULE_STEER_MOTOR_ID,
                7,
                OFFSETS[Module.RR.number],
                REAR_RIGHT_DRIVE_INVERTED,
                REAR_RIGHT_ANGLE_INVERTED,
                REAR_RIGHT_MOTION_MAGIC_CONFIGS);
    }

    public static SwerveDrive getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new SwerveDrive();
        }
        return INSTANCE;
    }

    /**
     * Gets the kinematics of the swerve.
     *
     * @return the kinematics of the swerve.
     */
    public SwerveDriveKinematics getKinematics() {
        return mKinematics;
    }

    /**
     * Resets the odometry of the robot to a specified pose. This is usually used
     * in the autonomous period.
     *
     * @param pose the pose to reset the odometry to.
     */
    public void resetOdometry(Pose2d pose) {
        mOdometry.resetPosition(pose.getRotation(), swerveModulePositions, pose);
    }

    /**
     * Gets the current pose of the robot.
     *
     * @return the current pose of the robot.
     */
    public Pose2d getPose() {
        return mOdometry.getPoseMeters();
    }


    /**
     * This is the main drive function. Any drive signal given here will be carried out in the periodic.
     *
     * @param driveSignal the drive signal to process.
     */
    public void drive(DriveSignal driveSignal) {
        if (Utils.speedsEpsilonEquals(driveSignal.speeds())) {
            stop();
            return;
        }

        var speeds = driveSignal.fieldOriented ? ChassisSpeeds.fromFieldRelativeSpeeds(
                driveSignal.vx,
                driveSignal.vy,
                driveSignal.omega,
                Gyroscope.getInstance().getYaw()) : driveSignal.speeds();
        loggerInputs.setpoint = Utils.chassisSpeedsToArray(speeds);
        swerveModuleStates = mKinematics.toSwerveModuleStates(speeds, driveSignal.centerOfRotation);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_VELOCITY_AUTO);

        mFrontLeft.set(swerveModuleStates[Module.FL.number].speedMetersPerSecond / MAX_VELOCITY_AUTO,
                swerveModuleStates[Module.FL.number].angle);
        mFrontRight.set(swerveModuleStates[Module.FR.number].speedMetersPerSecond / MAX_VELOCITY_AUTO,
                swerveModuleStates[Module.FR.number].angle);
        mRearLeft.set(swerveModuleStates[Module.RL.number].speedMetersPerSecond / MAX_VELOCITY_AUTO,
                swerveModuleStates[Module.RL.number].angle);
        mRearRight.set(swerveModuleStates[Module.RR.number].speedMetersPerSecond / MAX_VELOCITY_AUTO,
                swerveModuleStates[Module.RR.number].angle);
    }

    /**
     * Sets the states of the modules directly.
     *
     * @param states are the states to set.
     */
    public void setStates(SwerveModuleState[] states) {
        swerveModuleStates = states;

        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_AUTO);

        mFrontLeft.set(swerveModuleStates[Module.FL.number].speedMetersPerSecond / MAX_VELOCITY_AUTO,
                swerveModuleStates[Module.FL.number].angle);
        mFrontRight.set(swerveModuleStates[Module.FR.number].speedMetersPerSecond / MAX_VELOCITY_AUTO,
                swerveModuleStates[Module.FR.number].angle);
        mRearLeft.set(swerveModuleStates[Module.RL.number].speedMetersPerSecond / MAX_VELOCITY_AUTO,
                swerveModuleStates[Module.RL.number].angle);
        mRearRight.set(swerveModuleStates[Module.RR.number].speedMetersPerSecond / MAX_VELOCITY_AUTO,
                swerveModuleStates[Module.RR.number].angle);
    }

    /**
     * Locks the swerve so that it will be difficult to move.
     */
    public void lock() {
        mFrontLeft.set(0, Rotation2d.fromDegrees(45));
        mFrontRight.set(0, Rotation2d.fromDegrees(135));
        mRearLeft.set(0, Rotation2d.fromDegrees(315));
        mRearRight.set(0, Rotation2d.fromDegrees(225));
    }

    @Override
    public void updateInputs() {
        loggerInputs.pose = Utils.pose2dToArray(getPose());
        loggerInputs.speeds = Utils.chassisSpeedsToArray(mKinematics.toChassisSpeeds(
                mFrontLeft.getState(),
                mFrontRight.getState(),
                mRearLeft.getState(),
                mRearRight.getState()
        ));
    }

    @Override
    public String getSubsystemName() {
        return "SwerveDrive";
    }

    public ChassisSpeeds getSpeeds() {
        return Utils.arrayToChassisSpeeds(loggerInputs.speeds);
    }

    public void stop() {
        swerveModuleStates = mKinematics.toSwerveModuleStates(new ChassisSpeeds());
        mFrontLeft.stop();
        mFrontRight.stop();
        mRearLeft.stop();
        mRearRight.stop();
    }

    public void vroom() {
        mFrontLeft.vroom();
        mFrontRight.vroom();
        mRearLeft.vroom();
        mRearRight.vroom();
    }

    public SwerveModule[] getModules() {
        return new SwerveModule[] {
                mFrontLeft,
                mFrontRight,
                mRearLeft,
                mRearRight
        };
    }

    @Override
    public void periodic() {
        swerveModulePositions = new SwerveModulePosition[]{
                mFrontLeft.getPosition(),
                mFrontRight.getPosition(),
                mRearLeft.getPosition(),
                mRearRight.getPosition()
        };

        mOdometry.update(Gyroscope.getInstance().getYaw(), swerveModulePositions);

        SmartDashboard.putString("Encoder Positions", "{" +
                mFrontLeft.getEncoderTicks() + ", " +
                mFrontRight.getEncoderTicks() + ", " +
                mRearLeft.getEncoderTicks() + ", " +
                mRearRight.getEncoderTicks() + "}");

    }

    public enum Module {
        FL(0),
        FR(1),
        RL(2),
        RR(3);

        public final int number;

        Module(int number) {
            this.number = number;
        }
    }
}
