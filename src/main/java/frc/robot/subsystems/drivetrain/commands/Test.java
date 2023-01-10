package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class Test extends CommandBase {
    private final SwerveDrive swerveDrive = Robot.swerveSubsystem;
    private final Timer timer = new Timer();

    public Test() {
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        Robot.gyroscope.resetAngle();
        timer.start();
        timer.reset();
    }

    @Override
    public void execute() {
        swerveDrive.drive(new ChassisSpeeds(0.25, 0, 0.8));
    }

    @Override
    public boolean isFinished() {
        return timer.get() > 1;
    }
}
