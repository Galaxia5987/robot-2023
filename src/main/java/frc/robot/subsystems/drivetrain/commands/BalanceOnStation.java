package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gyroscope.Gyroscope;

public class BalanceOnStation extends CommandBase {
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final Gyroscope gyroscope = Gyroscope.getInstance();

    private final ProfiledPIDController controller = new ProfiledPIDController(
            SwerveConstants.CHARGING_STATION_BALANCE_Kp,
            SwerveConstants.CHARGING_STATION_BALANCE_Ki,
            SwerveConstants.CHARGING_STATION_BALANCE_Kd,
            SwerveConstants.CHARGING_STATION_BALANCE_CONSTRAINTS
    );

    public BalanceOnStation() {
        addRequirements(swerveDrive, gyroscope);
    }

    @Override
    public void initialize() {
        controller.setTolerance(Math.toRadians(2));
    }

    @Override
    public void execute() {
        var angles = gyroscope.getAll();
        double absoluteAngle = angles.getAngle();
        double vx = controller.calculate(absoluteAngle, 0);
        vx = Math.copySign(SwerveConstants.CHARGING_STATION_BALANCE_Kf, vx);
        swerveDrive.drive(new DriveSignal(vx, 0, 0, new Translation2d(), true));
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.stop();
    }

    @Override
    public boolean isFinished() {
        return controller.atSetpoint();
    }
}
