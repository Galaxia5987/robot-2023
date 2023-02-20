package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.utils.Utils;

public class IntakeDefaultCommand extends CommandBase {
    private final Intake intake = Intake.getInstance();
    private boolean lastIntakeNotAtSetpoint = false;

    public IntakeDefaultCommand() {
        addRequirements(intake);
    }

    @Override
    public void execute() {
        boolean intakeNotAtSetpoint = !Utils.epsilonEquals(intake.getAngle(), 0, 2);

        if (intakeNotAtSetpoint) {
            intake.setAnglePower(IntakeConstants.ANGLE_MOTOR_POWER);
        }
        if (intake.getCurrent() > IntakeConstants.MAX_CURRENT) {
            intake.setAnglePower(0);
        }

        lastIntakeNotAtSetpoint = intakeNotAtSetpoint;
    }
}
