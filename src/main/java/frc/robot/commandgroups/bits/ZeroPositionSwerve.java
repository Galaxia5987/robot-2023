package frc.robot.commandgroups.bits;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;

import java.util.Arrays;

public class ZeroPositionSwerve extends CommandBase {
    private final SwerveDrive swerve;
    private final Timer timer = new Timer();
    private final SwerveModuleState[] zeroStates = new SwerveModuleState[4];

    public ZeroPositionSwerve(SwerveDrive swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timer.start();
        timer.reset();
        Arrays.fill(zeroStates, new SwerveModuleState());
    }

    @Override
    public void execute() {
        swerve.setStates(zeroStates);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(1);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        swerve.stop();
    }
}

