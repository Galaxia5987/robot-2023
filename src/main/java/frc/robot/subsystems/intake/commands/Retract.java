package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class Retract extends CommandBase {
    private final Intake intake = Intake.getInstance();
    private final Mode mode;

    public Retract(Mode mode) {
        this.mode = mode;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (mode == Mode.UP) {
            intake.setAnglePower(IntakeConstants.ANGLE_MOTOR_POWER);
        } else {
            intake.setAnglePower(-IntakeConstants.ANGLE_MOTOR_POWER);
        }
    }

    @Override
    public boolean isFinished() {
        return intake.getCurrent() >= IntakeConstants.MAX_CURRENT;
    }

    @Override
    public void end(boolean interrupted) {
        if (mode == Mode.UP) {
            intake.resetEncoder();
        }
        intake.setAnglePower(0);
    }

    public enum Mode {
        UP,
        DOWN
    }
}
