package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.utils.controllers.PIDFController;

public class AdjustToTargetSmart extends CommandBase {
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final Limelight limelight = Limelight.getInstance();
    private final Gyroscope gyroscope = Gyroscope.getInstance();


    private final PIDFController xController = new PIDFController(SwerveConstants.TARGET_XY_Kp, SwerveConstants.TARGET_XY_Ki, SwerveConstants.TARGET_XY_Kd, SwerveConstants.TARGET_XY_Kf);
    private final PIDFController yController = new PIDFController(SwerveConstants.TARGET_XY_Kp, SwerveConstants.TARGET_XY_Ki, SwerveConstants.TARGET_XY_Kd, SwerveConstants.TARGET_XY_Kf);
    private final PIDFController rotationController = new PIDFController(SwerveConstants.TARGET_ROTATION_Kp, SwerveConstants.TARGET_ROTATION_Ki, SwerveConstants.TARGET_ROTATION_Kd, SwerveConstants.TARGET_ROTATION_Kf);

    public AdjustToTargetSmart() {
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
    }
}