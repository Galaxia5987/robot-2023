package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.utils.ui.ButtonMap;

public class HolonomicDrive extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final Gyroscope gyroscope;
    private final ButtonMap buttonMap;

    private final SlewRateLimiter forwardLimiter = new SlewRateLimiter(SwerveConstants.XY_SLEW_RATE_LIMIT);
    private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(SwerveConstants.XY_SLEW_RATE_LIMIT);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(SwerveConstants.ROTATION_SLEW_RATE_LIMIT);

    public HolonomicDrive(SwerveDrive swerveDrive, Gyroscope gyroscope, ButtonMap buttonMap) {
        this.swerveDrive = swerveDrive;
        this.gyroscope = gyroscope;
        this.buttonMap = buttonMap;
    }

    @Override
    public void execute() {
        // TODO: Fill these in
        if (buttonMap.getDoubleSubstation()) {

        } else if (buttonMap.getChargeStation()) {

        } else if (buttonMap.getIntakeCone()) {

        } else if (buttonMap.getIntakeCube()) {

        } else if (buttonMap.getScoreGamePiece()) {

        } else if (buttonMap.getSingleSubstation()) {

        } else {
            swerveDrive.drive(buttonMap.defaultDriveSignal(
                    forwardLimiter, strafeLimiter, rotationLimiter));
        }
    }
}
