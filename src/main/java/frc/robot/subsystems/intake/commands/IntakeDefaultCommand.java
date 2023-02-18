package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class IntakeDefaultCommand extends CommandBase {
    private final Intake intake = Intake.getInstance();

    private double angle;
    private boolean finishedUpwardMotion = false;

    public IntakeDefaultCommand() {
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setAnglePower(IntakeConstants.ANGLE_MOTOR_POWER);
    }

    @Override
    public void execute() {
        if (!finishedUpwardMotion && intake.getCurrent() >= IntakeConstants.MAX_CURRENT) {
            finishedUpwardMotion = true;
            intake.setAnglePower(0);
            angle = intake.getAngle() - 30;
        }
        if (finishedUpwardMotion) {
            intake.setAngle(angle);
        }

        System.out.println(finishedUpwardMotion);
    }
}
