package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.BeamBreaker;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class Retract extends CommandBase {
    private final Intake intake = Intake.getInstance();
    private final boolean state;

    public Retract(boolean state) {
        this.state = state;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (state){
            intake.setAnglePower(IntakeConstants.ANGLE_MOTOR_POWER);
        } else {
            intake.setAnglePower(-IntakeConstants.ANGLE_MOTOR_POWER);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.setAnglePower(0);
    }

    @Override
    public boolean isFinished() {
        return intake.getCurrent() >= IntakeConstants.MAX_CURRENT;
    }
}
