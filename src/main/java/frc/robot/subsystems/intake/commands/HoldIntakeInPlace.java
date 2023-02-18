package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;

public class HoldIntakeInPlace extends CommandBase {
    private final Intake intake = Intake.getInstance();
    private double angle;

    public HoldIntakeInPlace() {
    }

    @Override
    public void initialize() {
        angle = intake.getAngle();
    }

    @Override
    public void execute() {
        intake.setAngle(angle);
    }
}
