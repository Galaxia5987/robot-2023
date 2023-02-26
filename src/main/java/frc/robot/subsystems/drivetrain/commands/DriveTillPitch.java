package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gyroscope.Gyroscope;

public class DriveTillPitch extends CommandBase {
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final Gyroscope gyroscope = Gyroscope.getInstance();

    private final double desiredPitch;

    public DriveTillPitch(double desiredPitch) {
        this.desiredPitch = desiredPitch;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        var signal = new DriveSignal(
                2,
                0,
                SwerveConstants.AUTO_ROTATION_Kf,
                new Translation2d(),
                true
        );

        swerveDrive.drive(signal);
    }

    @Override
    public boolean isFinished() {
        return gyroscope.getPitch().getDegrees() >= desiredPitch;
    }
}
