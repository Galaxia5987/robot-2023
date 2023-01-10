package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.utils.Utils;
import frc.robot.utils.controllers.PIDFController;

public class BalanceOnStation extends CommandBase {
    private final SwerveDrive swerveDrive = Robot.swerveSubsystem;
    private final Gyroscope gyroscope = Robot.gyroscope;

    private final PIDFController controller = new PIDFController(
            Constants.SwerveDrive.CHARGING_STATION_BALANCE_Kp,
            Constants.SwerveDrive.CHARGING_STATION_BALANCE_Ki,
            Constants.SwerveDrive.CHARGING_STATION_BALANCE_Kd,
            Constants.SwerveDrive.CHARGING_STATION_BALANCE_Kf
    );

    public BalanceOnStation() {
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        controller.setTolerance(Math.toRadians(2));
    }

    @Override
    public void execute() {
        var angles = gyroscope.getYawPitchRoll();
        double absoluteAngle = Utils.relativeAnglesToAbsolutePitch(
                angles.yaw(),
                angles.pitch(),
                angles.roll()
        );
        double vx = controller.calculate(absoluteAngle, 0);
        swerveDrive.drive(vx, 0, 0);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.drive(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return controller.atSetpoint();
    }
}
