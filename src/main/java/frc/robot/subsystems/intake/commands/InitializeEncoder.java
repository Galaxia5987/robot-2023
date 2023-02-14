package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.BeamBreaker;
import frc.robot.subsystems.intake.Intake;

public class InitializeEncoder extends CommandBase {
    private final Intake intake = Intake.getInstance();
    private final BeamBreaker beamBreaker = BeamBreaker.getInstance();

    public InitializeEncoder() {
        addRequirements(intake);
    }


    @Override
    public void execute() {
        intake.setAnglePower(0.2);
    }

    @Override
    public void end(boolean interrupted) {
        intake.resetEncoder();
    }
}
