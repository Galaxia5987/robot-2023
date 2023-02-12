package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.BeamBreaker;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

public class Feed extends CommandBase {
    private final Intake intake = Intake.getInstance();
    private final BeamBreaker beamBreaker = BeamBreaker.getInstance();
    private final double power;

    public Feed(double power) {
        this.power = power;
        addRequirements(intake);
    }

    /**
     * open the retractor for cubes
     */
    @Override
    public void initialize() {
        intake.setAngle(IntakeConstants.ANGLE_DOWN);
    }

    /**
     * start the motor of the Intake
     */
    @Override
    public void execute() {
        intake.setPower(power);
    }


    @Override
    public boolean isFinished() {
        return beamBreaker.isBeamBlocked();
    }

    @Override
    public void end(boolean interrupted) {
        intake.setAngle(IntakeConstants.ANGLE_UP);
        intake.setPower(0);
    }
}
