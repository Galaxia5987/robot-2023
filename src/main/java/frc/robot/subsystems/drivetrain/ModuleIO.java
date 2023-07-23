package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    void updateInputs(SwerveModuleInputs inputs);

    default double getAngle() {
        return 0;
    }

    void setAngle(double angle);

    default double getVelocity() {
        return 0;
    }

    void setVelocity(double velocity);

    default SwerveModulePosition getModulePosition() {
        return null;
    }

    default void updateOffset(double offset) {
    }

    default void neutralOutput() {
    }

    default boolean encoderConnected() {
        return false;
    }

    default void checkModule(){}

    @AutoLog
    class SwerveModuleInputs {
        public double driveMotorVelocity = 0;
        public double driveMotorVelocitySetpoint = 0;
        public double driveMotorSupplyCurrent = 0;
        public double driveMotorStatorCurrent = 0;
        public double driveMotorSupplyCurrentOverTime = 0;
        public double driveMotorStatorCurrentOverTime = 0;
        public double driveMotorPosition = 0;
        public double driveMotorAppliedVoltage = 0;

        public double angle = 0;
        public double angleSetpoint = 0;
        public double absolutePosition = 0;
        public double angleMotorVelocity = 0;
        public double angleMotorSupplyCurrent = 0;
        public double angleMotorStatorCurrent = 0;
        public double angleMotorSupplyCurrentOverTime = 0;
        public double angleMotorStatorCurrentOverTime = 0;
        public double angleMotorPosition = 0;
        public double angleMotorAppliedVoltage = 0;

        public double moduleDistance = 0;
    }
}
