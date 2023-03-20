package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gyroscope.Gyroscope;

public class DriveTillPitchAccurate extends CommandBase {
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final Gyroscope gyroscope = Gyroscope.getInstance();

    private final double desiredPitch;
    private final double xVelocity;
    private double currentPitch;
    private double startingPitch;

    private int counter = 0;

    public DriveTillPitchAccurate(double desiredPitch, double xVelocity) {
        this.desiredPitch = desiredPitch;
        this.xVelocity = xVelocity;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        startingPitch = gyroscope.getPitch().getDegrees();
    }

    @Override
    public void execute() {
        if (((startingPitch - desiredPitch) > 0 && (currentPitch-desiredPitch) < 0) || ((startingPitch - desiredPitch) < 0 && (currentPitch - desiredPitch) > 0)) {
            counter++;
            startingPitch = gyroscope.getPitch().getDegrees();
        }

        currentPitch = gyroscope.getPitch().getDegrees();
        double vx = xVelocity * Math.signum(desiredPitch - currentPitch);
        var signal = new DriveSignal(
                vx,
                0,
                0,
                new Translation2d(),
                true
        );

        swerveDrive.drive(signal);

        System.out.println("Hey there sexy");
    }

    @Override
    public boolean isFinished() {
        return counter > 3;
    }
}
