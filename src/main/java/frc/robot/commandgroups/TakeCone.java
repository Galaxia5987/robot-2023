package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.ArmAxisControl;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.leds.LedConstants;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.leds.command.Blink;

public class TakeCone extends SequentialCommandGroup {

    public TakeCone(){
        Gripper gripper = Gripper.getInstance();
        Leds leds = Leds.getInstance();
        addCommands(
                new InstantCommand(() -> leds.setBlinkTime(LedConstants.FAST_BLINK_TIME)),
                new ArmAxisControl(1, 0.02, 0)
                        .until(() -> gripper.getDistance() < ArmConstants.FEEDER_DISTANCE),
                new InstantCommand(() ->leds.setBlink(true, LedConstants.kGalaxiaBlue))
        );
    }
}
