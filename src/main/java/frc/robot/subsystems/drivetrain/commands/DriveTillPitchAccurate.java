package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.utils.Utils;

public class DriveTillPitchAccurate extends CommandBase {
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final Gyroscope gyroscope = Gyroscope.getInstance();

    private final double desiredPitch;
    private final double xVelocity;
    double currentPitch = gyroscope.getPitch().getDegrees();
    double startingPitch;

    public DriveTillPitchAccurate(double desiredPitch, double xVelocity) {
        this.desiredPitch = desiredPitch;
        this.xVelocity = xVelocity*Math.signum(desiredPitch-startingPitch);
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        startingPitch = currentPitch;
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
       return ((startingPitch - desiredPitch) > 0 && (startingPitch - currentPitch) < 0 )|| ((startingPitch - desiredPitch) < 0 && (startingPitch - currentPitch) > 0);

}}
