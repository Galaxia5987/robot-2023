package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class SwerveModule extends SubsystemBase {

    private final SwerveModuleInputsAutoLogged loggerInputs = new SwerveModuleInputsAutoLogged();

    private final ModuleIO io;

    private final int number;

    private SwerveModuleState currentModuleState = new SwerveModuleState();

    public SwerveModule(ModuleIO io, int number) {
        this.io = io;
        this.number = number;
    }

    /**
     * Sets the module to a desired module state.
     *
     * @param moduleState A module state to set the module to.
     */
    public void setModuleState(SwerveModuleState moduleState) {
        moduleState = SwerveModuleState.optimize(moduleState, new Rotation2d(loggerInputs.angle));
        io.setVelocity(moduleState.speedMetersPerSecond);
        io.setAngle(moduleState.angle.getRadians());
    }

    public void setModuleState(double speed, Rotation2d angle) {
        setModuleState(new SwerveModuleState(speed, angle));
    }

    /**
     * Gets the state of a module.
     *
     * @return The state of a module.
     */
    public SwerveModuleState getModuleState() {
        return currentModuleState;
    }

    /**
     * Gets the position of the module.
     *
     * @return Position of the module.
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
                loggerInputs.moduleDistance, new Rotation2d(loggerInputs.angle)
        );
    }

    /**
     * Gets the stator current of both motors combined.
     *
     * @return Sum of the drive motor stator current and angle motor stator current. [amps]
     */
    public double getStatorCurrent() {
        return loggerInputs.driveMotorStatorCurrent + loggerInputs.angleMotorStatorCurrent;
    }

    /**
     * Gets the supply current of both motors combined.
     *
     * @return Sum of the drive motor supply current and angle motor supply current. [amps]
     */
    public double getSupplyCurrent() {
        return loggerInputs.driveMotorSupplyCurrent + loggerInputs.angleMotorSupplyCurrent;
    }

    /**
     * Gets the position of the absolute encoder.
     *
     * @return Position of the absolute encoder. [sensor ticks]
     */
    public double getPosition() {
        return loggerInputs.absolutePosition;
    }

    /**
     * Updates the position of the angle motor with an offset and an absolute encoder.
     *
     * @param offset The offset to update the angle motor's position. [sensor ticks]
     */
    public void updateOffset(double offset) {
        io.updateOffset(offset);
    }

    public void neutralOutput() {
        io.neutralOutput();
    }

    public boolean encoderConnected() {
        return io.encoderConnected();
    }

    public void checkModule(){
        io.checkModule();
    }

    @Override
    public void periodic() {
        currentModuleState = new SwerveModuleState(
                io.getVelocity(), new Rotation2d(io.getAngle())
        );

        io.updateInputs(loggerInputs);

        Logger.getInstance().recordOutput("SwerveDrive/currentModuleState" + number, currentModuleState);

        Logger.getInstance().processInputs("module_" + number, loggerInputs);
    }
}
