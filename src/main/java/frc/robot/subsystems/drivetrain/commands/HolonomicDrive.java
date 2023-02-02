package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.utils.ui.ButtonMap;

public class HolonomicDrive extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final ButtonMap buttonMap;

    private final SlewRateLimiter forwardLimiter = new SlewRateLimiter(SwerveConstants.XY_SLEW_RATE_LIMIT);
    private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(SwerveConstants.XY_SLEW_RATE_LIMIT);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(SwerveConstants.ROTATION_SLEW_RATE_LIMIT);
    private final DriveSignal signal = new DriveSignal(0, 0, 0, new Translation2d(), true);

    public HolonomicDrive(SwerveDrive swerveDrive, ButtonMap buttonMap) {
        this.swerveDrive = swerveDrive;
        this.buttonMap = buttonMap;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        // TODO: Fill these in
        if (buttonMap.getDoubleSubstation()) {

        } else if (buttonMap.getChargeStation()) {

        } else if (buttonMap.getIntakeCone()) {

        } else if (buttonMap.getIntakeCube()) {

        } else if (buttonMap.getSingleSubstation()) {

        } else if (buttonMap.lock()) {
            swerveDrive.lock();
            signal.vx = 0;
            signal.vy = 0;
            signal.omega = 0;
        } else {
            buttonMap.defaultDriveSignal(signal, forwardLimiter, strafeLimiter, rotationLimiter);
            swerveDrive.drive(signal);
        }
    }

}
