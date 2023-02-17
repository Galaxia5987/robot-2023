package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.utils.math.ArmPath;

import java.io.File;

public class GetArmOutOfRobot extends CommandBase {
    private final Arm arm = Arm.getInstance();

    private final ArmPath armPath = new ArmPath(new File("home/lvuser/arm_path.csv"));
    private final Timer timer = new Timer();

    public GetArmOutOfRobot() {
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        timer.start();
        timer.reset();
    }

    @Override
    public void execute() {
        double t = timer.get();
        var shoulderSetpoint = armPath.getShoulderAngle(t);
        var elbowSetpoint = armPath.getElbowAngle(t);

        if (shoulderSetpoint.isPresent() && elbowSetpoint.isPresent()) {
            arm.setShoulderJointAngle(shoulderSetpoint.get().value);
            arm.setElbowJointAngle(elbowSetpoint.get().value);
        }
    }
}
