package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.utils.Utils;

public class DriveTillPitch extends CommandBase {
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final Gyroscope gyroscope = Gyroscope.getInstance();

    private final double desiredPitch;
    private final int xVelocity;

    public DriveTillPitch(double desiredPitch, int xVelocity) {
        this.desiredPitch = desiredPitch;
        this.xVelocity = xVelocity;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        var signal = new DriveSignal(
                xVelocity,
                0,
                0,
                new Translation2d(),
                true
        );

        swerveDrive.drive(signal);
    }

    @Override
    public boolean isFinished() {
        if (desiredPitch > 0) {
            return gyroscope.getPitch().getDegrees() >= desiredPitch;
        } else if (desiredPitch < 0) {
            return gyroscope.getPitch().getDegrees() <= desiredPitch;
        } else {
            return Utils.epsilonEquals(gyroscope.getPitch().getDegrees(), 0, 1);
        }
        // Math.abs(gyroscope.getPitch().getDegrees()) >= Math.abs(desiredPitch); TODO: alternative for the first two if's above.
    }
}
