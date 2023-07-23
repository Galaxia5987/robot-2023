package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.utils.Utils;
import frc.robot.utils.controllers.DieterController;

public class DriveTillPitch extends CommandBase {
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final Gyroscope gyroscope = Gyroscope.getInstance();

    private final double desiredPitch;
    private final double xVelocity;

    private final DieterController yawController = new DieterController(3, 0, 0, 0);

    public DriveTillPitch(double desiredPitch, double xVelocity) {
        this.desiredPitch = desiredPitch;
        this.xVelocity = xVelocity;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        swerveDrive.drive(
                xVelocity,
                0,
                yawController.calculate(gyroscope.getYaw().getRadians(), 0),
                true
        );
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
//         return Math.abs(gyroscope.getPitch().getDegrees()) >= Math.abs(desiredPitch);
    }
}
