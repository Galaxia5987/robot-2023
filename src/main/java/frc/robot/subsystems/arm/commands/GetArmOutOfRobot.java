package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.utils.math.ArmLinearProfile;

public class GetArmOutOfRobot extends CommandBase {
    private final Arm arm = Arm.getInstance();

    private ArmLinearProfile currentProfile;
    private final Timer timer = new Timer();

    public GetArmOutOfRobot() {
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        var currentPosition = arm.getEndPosition();
        currentProfile = new ArmLinearProfile(
                new TrapezoidProfile.Constraints(0.5, 0.25),
                ArmConstants.ARM_OUT_OF_ROBOT_POINT1,
                new ArmLinearProfile.Waypoint(currentPosition.getX(), currentPosition.getY(), 0, 0)).setNext(
                        new ArmLinearProfile(
                                new TrapezoidProfile.Constraints(0.5, 0.25),
                                ArmConstants.ARM_OUT_OF_ROBOT_POINT2,
                                ArmConstants.ARM_OUT_OF_ROBOT_POINT1));

        timer.start();
        timer.reset();
    }

    @Override
    public void execute() {
        double t = timer.get();

        if (currentProfile.hasFinished(t)) {
            currentProfile = currentProfile.getNext();
            timer.reset();
        }

        if (currentProfile != null) {
            arm.setEndPosition(currentProfile.calculate(t));
        }
    }

    @Override
    public boolean isFinished() {
        return currentProfile == null;
    }
}
