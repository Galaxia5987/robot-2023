package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.utils.Utils;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends SubsystemBase {
    private static SwerveDrive INSTANCE = null;
    private SwerveDriveInputsAutoLogged loggerInputs = new SwerveDriveInputsAutoLogged();

    private final GyroIO gyro;
    private final frc.robot.subsystems.drivetrain.SwerveModule[] modules = new frc.robot.subsystems.drivetrain.SwerveModule[4]; //FL, FR, RL, RR
    private SwerveModuleState[] currentModuleStates = new SwerveModuleState[4];
    private SwerveModuleState[] desiredModuleStates = new SwerveModuleState[4];

    private final PIDController pidController = new PIDController(frc.robot.subsystems.drivetrain.SwerveConstants.OMEGA_kP, frc.robot.subsystems.drivetrain.SwerveConstants.OMEGA_kI, frc.robot.subsystems.drivetrain.SwerveConstants.OMEGA_kD);
    private boolean shouldKeepAngle = false;

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            frc.robot.subsystems.drivetrain.SwerveConstants.wheelPositions[0],
            frc.robot.subsystems.drivetrain.SwerveConstants.wheelPositions[1],
            frc.robot.subsystems.drivetrain.SwerveConstants.wheelPositions[2],
            frc.robot.subsystems.drivetrain.SwerveConstants.wheelPositions[3]
    );

    private SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

    private final SwerveDriveOdometry odometry;

    private SwerveDrive() {
        if (Robot.isReal()) {
            for (int i = 0; i < modules.length; i++) {
                ModuleIO io = new ModuleIOReal(
                        Ports.SwerveDrive.DRIVE_IDS[i],
                        Ports.SwerveDrive.ANGLE_IDS[i],
                        Ports.SwerveDrive.ENCODER_IDS[i],
                        frc.robot.subsystems.drivetrain.SwerveConstants.motionMagicConfigs[i],
                        i + 1);

                modules[i] = new frc.robot.subsystems.drivetrain.SwerveModule(io, i + 1);
            }

            gyro = new GyroIOReal();
        } else {
            for (int i = 0; i < modules.length; i++) {
                ModuleIO io = new ModuleIOSim();
                modules[i] = new frc.robot.subsystems.drivetrain.SwerveModule(io, i + 1);
            }

            gyro = new GyroIOSim();
        }
        pidController.enableContinuousInput(0, Math.PI * 2);
        pidController.setTolerance(Math.toRadians(3));

        updateModulePositions();
        odometry = new SwerveDriveOdometry(
                kinematics, new Rotation2d(getYaw()),
                modulePositions
        );
    }

    public static SwerveDrive getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new SwerveDrive();
        }
        return INSTANCE;
    }

    public SwerveDriveKinematics getKinematics(){
        return kinematics;
    }

    /**
     * Updates the offset for the gyro.
     *
     * @param angle The desired angle. [rad]
     */
    public void resetGyro(double angle) {
        gyro.resetGyro(angle);
    }

    public void resetGyro() {
        resetGyro(0);
    }

    /**
     * Gets the raw yaw reading from the gyro.
     *
     * @return Yaw angle reading from gyro. [rad]
     */
    public double getRawYaw() {
        return gyro.getRawYaw();
    }

    /**
     * Gets the yaw reading from the gyro with the calculated offset.
     *
     * @return Yaw angle with offset. [rad]
     */
    public double getYaw() {
        return gyro.getYaw();
    }

    /**
     * Sets the module states to the desired module states.
     *
     * @param desiredModuleStates The desired module states to set the modules to.
     */
    public void setModuleStates(SwerveModuleState[] desiredModuleStates) {
        Logger.getInstance().recordOutput("SwerveDrive/desiredModuleStates", desiredModuleStates);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, frc.robot.subsystems.drivetrain.SwerveConstants.MAX_X_Y_VELOCITY);
        for (int i = 0; i < modules.length; i++) {
            modules[i].setModuleState(desiredModuleStates[i]);
        }
    }

    /**
     * Updates each module position with an offset and an absolute encoder.
     *
     * @param offsets Offsets for each of the modules. [sensor ticks]
     */
    public void updateOffsets(double[] offsets) {
        for (int i = 0; i < modules.length; i++) {
            modules[i].updateOffset(offsets[i]);
        }
    }

    public void updateModulePositions() {
        for (int i = 0; i < modulePositions.length; i++) {
            modulePositions[i] = modules[i].getModulePosition();
        }
    }

    public Pose2d getBotPose() {
        return odometry.getPoseMeters();
    }

    public void resetPose() {
        odometry.resetPosition(new Rotation2d(getYaw()), modulePositions, new Pose2d());
    }

    public void resetPose(Pose2d pose){
        odometry.resetPosition(new Rotation2d(getYaw()), modulePositions, pose);
    }

    public boolean encodersConnected() {
        boolean connected = true;
        for (int i = 0; i < 4; i++) {
            connected &= modules[i].encoderConnected();
        }
        return connected;
    }

    public void lock() {
        modules[0].setModuleState(0, Rotation2d.fromDegrees(45));
        modules[1].setModuleState(0, Rotation2d.fromDegrees(135));
        modules[2].setModuleState(0, Rotation2d.fromDegrees(315));
        modules[3].setModuleState(0, Rotation2d.fromDegrees(225));
    }

    public void checkSwerve(){
        for (int i = 0; i < 4; i++) {
            modules[i].checkModule();
        }
    }

    /**
     * Sets the correct module states from desired chassis speeds.
     *
     * @param chassisSpeeds Desired chassis speeds.
     * @param fieldOriented Should the drive be field oriented.
     */
    public void drive(ChassisSpeeds chassisSpeeds, boolean fieldOriented) {
        loggerInputs.desiredSpeeds = Utils.chassisSpeedsToArray(chassisSpeeds);

//        if (fieldOriented){
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond,
                chassisSpeeds.omegaRadiansPerSecond,
                new Rotation2d(getYaw())
        );
//        }

        if (chassisSpeeds.equals(new ChassisSpeeds(0, 0, 0))) {
            for (SwerveModule module : modules) {
                module.neutralOutput();
            }
        }

        setModuleStates(kinematics.toSwerveModuleStates(chassisSpeeds));
    }

    /**
     * Sets the desired percentage of x, y and omega speeds for the swerve
     *
     * @param xOutput     percentage of the x speed
     * @param yOutput     percentage of the y speed
     * @param omegaOutput percentage of the omega speed
     */
    public void drive(double xOutput, double yOutput, double omegaOutput, boolean fieldOriented) {
        double angleFF = 0;
        if (Utils.epsilonEquals(omegaOutput, 0)) {
            double angle = getRawYaw();
            if (!shouldKeepAngle && Utils.epsilonEquals(loggerInputs.currentSpeeds[2], 0, 0.1)) {
                pidController.setSetpoint(angle);
                shouldKeepAngle = true;
            } else {
                angleFF = pidController.calculate(angle);
            }
        } else {
            shouldKeepAngle = false;
        }

        loggerInputs.angleFF = angleFF;
        loggerInputs.pidSetpoint = pidController.getSetpoint();
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                frc.robot.subsystems.drivetrain.SwerveConstants.MAX_X_Y_VELOCITY * xOutput,
                frc.robot.subsystems.drivetrain.SwerveConstants.MAX_X_Y_VELOCITY * yOutput,
                SwerveConstants.MAX_OMEGA_VELOCITY * omegaOutput + angleFF);

        drive(chassisSpeeds, fieldOriented);
    }

    public void periodic() {
        updateModulePositions();
        odometry.update(new Rotation2d(getYaw()), modulePositions);

        loggerInputs.botPose[0] = getBotPose().getX();
        loggerInputs.botPose[1] = getBotPose().getY();
        loggerInputs.botPose[2] = getBotPose().getRotation().getRadians();

        for (int i = 0; i < modules.length; i++) {
            currentModuleStates[i] = modules[i].getModuleState();
            loggerInputs.absolutePositions[i] = modules[i].getPosition();
        }

        Logger.getInstance().recordOutput("SwerveDrive/currentModuleSates", currentModuleStates);

        for (int i = 0; i < 3; i++) {
            loggerInputs.currentSpeeds[i] =
                    Utils.chassisSpeedsToArray(
                            kinematics.toChassisSpeeds(
                                    currentModuleStates[0],
                                    currentModuleStates[1],
                                    currentModuleStates[2],
                                    currentModuleStates[3]
                            ))[i];
        }

        loggerInputs.supplyCurrent =
                modules[0].getSupplyCurrent() + modules[1].getSupplyCurrent() + modules[2].getSupplyCurrent() + modules[3].getStatorCurrent();

        loggerInputs.statorCurrent =
                modules[0].getStatorCurrent() + modules[1].getStatorCurrent() + modules[2].getStatorCurrent() + modules[3].getStatorCurrent();

        loggerInputs.rawYaw = getRawYaw();
        loggerInputs.yaw = getYaw();
        gyro.updateInputs(loggerInputs);

        Logger.getInstance().processInputs("SwerveDrive", loggerInputs);
    }
}
