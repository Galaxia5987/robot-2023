package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import frc.robot.Robot;
import frc.robot.subsystems.LoggedSubsystem;
import frc.robot.utils.Utils;

import java.util.Arrays;

import static frc.robot.Constants.SwerveDrive.*;


public class SwerveDrive extends LoggedSubsystem<SwerveDriveLogInputs> {
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

    private boolean fieldOriented = true;

    public SwerveDrive() {
        super(new SwerveDriveLogInputs());

        mFrontLeft = new SwerveModule(
                Module.FL,
                FRONT_LEFT_MODULE_DRIVE_MOTOR_ID,
                FRONT_LEFT_MODULE_STEER_MOTOR_ID,
                3,
                OFFSETS[Module.FL.number],
                FRONT_LEFT_DRIVE_INVERTED,
                FRONT_LEFT_ANGLE_INVERTED,
                FRONT_LEFT_ANGLE_SENSOR_PHASE,
                FRONT_LEFT_MOTION_MAGIC_CONFIGS);
        mFrontRight = new SwerveModule(
                Module.FR,
                FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID,
                FRONT_RIGHT_MODULE_STEER_MOTOR_ID,
                2,
                OFFSETS[Module.FR.number],
                FRONT_RIGHT_DRIVE_INVERTED,
                FRONT_RIGHT_ANGLE_INVERTED,
                FRONT_RIGHT_ANGLE_SENSOR_PHASE,
                FRONT_RIGHT_MOTION_MAGIC_CONFIGS);
        mRearLeft = new SwerveModule(
                Module.RL,
                REAR_LEFT_MODULE_DRIVE_MOTOR_ID,
                REAR_LEFT_MODULE_STEER_MOTOR_ID,
                0,
                OFFSETS[Module.RL.number],
                REAR_LEFT_DRIVE_INVERTED,
                REAR_LEFT_ANGLE_INVERTED,
                REAR_LEFT_ANGLE_SENSOR_PHASE,
                REAR_LEFT_MOTION_MAGIC_CONFIGS);
        mRearRight = new SwerveModule(
                Module.RR,
                REAR_RIGHT_MODULE_DRIVE_MOTOR_ID,
                REAR_RIGHT_MODULE_STEER_MOTOR_ID,
                1,
                OFFSETS[Module.RR.number],
                REAR_RIGHT_DRIVE_INVERTED,
                REAR_RIGHT_ANGLE_INVERTED,
                REAR_RIGHT_ANGLE_SENSOR_PHASE,
                REAR_RIGHT_MOTION_MAGIC_CONFIGS);
    }

    public SwerveDriveKinematics getKinematics() {
        return mKinematics;
    }

    public void updateOdometry() {
        mOdometry.update(Robot.gyroscope.getYaw(), swerveModulePositions);
    }

    public void resetOdometry(Pose2d pose) {
        mOdometry.resetPosition(Robot.gyroscope.getYaw(), swerveModulePositions, pose);
    }

    public Pose2d getPose() {
        return mOdometry.getPoseMeters();
    }

    public void drive(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotation) {
        drive(chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond,
                chassisSpeeds.omegaRadiansPerSecond, centerOfRotation);
    }

    public void drive(double vx, double vy, double theta, Translation2d centerOfRotation) {
        if (Utils.epsilonEquals(vx, 0, 0.1 * MAX_VELOCITY_METERS_PER_SECOND) &&
                Utils.epsilonEquals(vy, 0, 0.1 * MAX_VELOCITY_METERS_PER_SECOND) &&
                Utils.epsilonEquals(theta, 0, 0.1 * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)) {
            stop();
            return;
        }

        swerveModuleStates = mKinematics.toSwerveModuleStates(fieldOriented ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        vx,
                        vy,
                        theta,
                        Robot.gyroscope.getYaw()) : new ChassisSpeeds(vx, vy, theta),
                centerOfRotation);
    }

    public void drive(double vx, double vy, double theta) {
        drive(vx, vy, theta, new Translation2d());
    }

    public void drive(ChassisSpeeds speeds) {
        drive(speeds, new Translation2d());
    }

    public void setFieldOriented(boolean fieldOriented) {
        this.fieldOriented = fieldOriented;
    }

    public void setStates(SwerveModuleState[] states) {
        swerveModuleStates = Arrays.copyOf(states, states.length);
    }

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

    @Override
    public void periodic() {
        swerveModulePositions = new SwerveModulePosition[]{
                mFrontLeft.getPosition(),
                mFrontRight.getPosition(),
                mRearLeft.getPosition(),
                mRearRight.getPosition()
        };

        updateOdometry();

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_VELOCITY_METERS_PER_SECOND);

        mFrontLeft.set(swerveModuleStates[Module.FL.number].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND,
                swerveModuleStates[Module.FL.number].angle);
        mFrontRight.set(swerveModuleStates[Module.FR.number].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND,
                swerveModuleStates[Module.FR.number].angle);
        mRearLeft.set(swerveModuleStates[Module.RL.number].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND,
                swerveModuleStates[Module.RL.number].angle);
        mRearRight.set(swerveModuleStates[Module.RR.number].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND,
                swerveModuleStates[Module.RR.number].angle);

        System.out.println("{" +
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
