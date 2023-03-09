package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.utils.Utils;
import frc.robot.utils.controllers.DieterController;

public class BalanceOnStation extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final Gyroscope gyroscope;
    private final Timer timer = new Timer();
    private final DieterController controller = new DieterController(
            SwerveConstants.CHARGING_STATION_BALANCE_Kp,
            SwerveConstants.CHARGING_STATION_BALANCE_Ki,
            SwerveConstants.CHARGING_STATION_BALANCE_Kd,
            SwerveConstants.CHARGING_STATION_BALANCE_Kf
    );
    private boolean atSetpoint = false;
    private boolean lastAtSetpoint = false;

    public BalanceOnStation() {
        this.swerveDrive = SwerveDrive.getInstance();
        this.gyroscope = Gyroscope.getInstance();
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        timer.start();
        timer.reset();
    }

    @Override
    public void execute() {
        double pitch = gyroscope.getPitch().getSin();
        double vx = controller.calculate(pitch, 0);
        atSetpoint = Utils.epsilonEquals(0, pitch, Rotation2d.fromDegrees(2).getSin());
        if (atSetpoint) {
            swerveDrive.stop();
        } else {
            swerveDrive.drive(new DriveSignal(vx, 0, 0, new Translation2d(), true));
        }

        if (!lastAtSetpoint && atSetpoint) {
            timer.reset();
        }

        lastAtSetpoint = atSetpoint;
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.stop();
    }

    @Override
    public boolean isFinished() {
        return atSetpoint && lastAtSetpoint && timer.get() > 2;
    }
}
