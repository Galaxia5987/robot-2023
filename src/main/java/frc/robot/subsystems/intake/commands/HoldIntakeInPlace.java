package frc.robot.subsystems.intake.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class HoldIntakeInPlace extends CommandBase {
    private final Intake intake = Intake.getInstance();
    private double angle;

    public HoldIntakeInPlace() {
        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (intake.switchedToDefaultCommand() || Robot.justEnabled()) {
            angle = intake.getAngle();
            angle = MathUtil.clamp(angle, IntakeConstants.ANGLE_DOWN, IntakeConstants.ANGLE_UP);
        }
        intake.setAngle(angle);
    }
}
