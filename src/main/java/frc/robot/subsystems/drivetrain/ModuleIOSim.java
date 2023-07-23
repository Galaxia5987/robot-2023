package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.utils.math.AngleUtil;
import frc.robot.utils.math.differential.Integral;

public class ModuleIOSim implements ModuleIO {
    private final FlywheelSim driveMotor;
    private final FlywheelSim angleMotor;

    private final PIDController angleFeedback;
    private final PIDController velocityFeedback;

    private double currentVelocity = 0;
    private double driveMotorAppliedVoltage = 0;
    private double angleMotorAppliedVoltage = 0;

    private double velocitySetpoint = 0;
    private double angleSetpoint = 0;

    private Integral currentAngle = new Integral(0, 0);
    private Integral moduleDistance = new Integral(0, 0);

    public ModuleIOSim() {
        driveMotor = new FlywheelSim(
                DCMotor.getFalcon500(1),
                1 / SwerveConstants.DRIVE_REDUCTION,
                SwerveConstants.DriveMotorMomentOfInertia);

        angleMotor = new FlywheelSim(
                DCMotor.getFalcon500(1),
                1 / SwerveConstants.ANGLE_REDUCTION,
                SwerveConstants.AngleMotorMomentOfInertia);

        angleFeedback = new PIDController(3.5, 0, 0, 0.02);
        velocityFeedback = new PIDController(0.5, 0, 0.00, 0.02);
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        driveMotor.update(0.02);
        angleMotor.update(0.02);

        currentAngle.update(angleMotor.getAngularVelocityRadPerSec());

        inputs.driveMotorAppliedVoltage = driveMotorAppliedVoltage;
        inputs.driveMotorVelocity = driveMotor.getAngularVelocityRadPerSec();
        inputs.driveMotorVelocitySetpoint = velocitySetpoint;

        inputs.angleMotorAppliedVoltage = angleMotorAppliedVoltage;
        inputs.angleMotorVelocity = angleMotor.getAngularVelocityRadPerSec();
        inputs.angleSetpoint = angleSetpoint;
        inputs.angle = AngleUtil.normalize(currentAngle.get());

        moduleDistance.update(inputs.driveMotorVelocity);
        inputs.moduleDistance = moduleDistance.get();
    }

    @Override
    public void setAngle(double angle) {
        angleSetpoint = angle;
        angleMotorAppliedVoltage = angleFeedback.calculate(MathUtil.angleModulus(currentAngle.get()), angle);
        angleMotor.setInputVoltage(angleMotorAppliedVoltage);
    }

    @Override
    public double getAngle() {
        return currentAngle.get();
    }

    @Override
    public void setVelocity(double velocity) {
        velocitySetpoint = velocity;
        currentVelocity = driveMotor.getAngularVelocityRadPerSec();
        driveMotorAppliedVoltage = velocityFeedback.calculate(currentVelocity, velocity);
        driveMotor.setInputVoltage(driveMotorAppliedVoltage);
    }

    @Override
    public double getVelocity() {
        return currentVelocity;
    }
}
