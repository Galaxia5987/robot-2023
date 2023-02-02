package frc.robot.subsystems.intake.Command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.BeamBreaker;
import frc.robot.subsystems.intake.ConstantsIntake;
import frc.robot.subsystems.intake.Intake;

public class Feed extends CommandBase {
    private final Intake intake;
    private final BeamBreaker beamBreaker;
    private final double power;

    public Feed(double power, Intake intake, BeamBreaker beamBreaker) {
        this.intake = intake;
        this.beamBreaker = beamBreaker;
        this.power = power;
        addRequirements(intake);
    }

    /**
     * open the retractor for cubes
     */
    @Override
    public void initialize() {
        intake.setAngle(ConstantsIntake.INTAKE_OPENED_POSITION);
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
        intake.setAngle(ConstantsIntake.INTAKE_CLOSED_POSITION);
        intake.setPower(0);
    }
}
