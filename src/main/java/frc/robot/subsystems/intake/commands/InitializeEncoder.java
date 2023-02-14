package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class InitializeEncoder extends CommandBase {
    private final Intake intake = Intake.getInstance();

    public InitializeEncoder() {
        addRequirements(intake);
    }

    @Override
    public boolean isFinished() {
        return intake.getCurrent() >= IntakeConstants.MAX_CURRENT;
    }

    @Override
    public void execute() {
        intake.setAnglePower(IntakeConstants.ANGLE_MOTOR_POWER);
    }

    @Override
    public void end(boolean interrupted) {
        intake.resetEncoder();
    }
}
