package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.ArmAxisControl;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.leds.LedConstants;
import frc.robot.subsystems.leds.Leds;

public class TakeCone extends SequentialCommandGroup {

    public TakeCone(){
        Gripper gripper = Gripper.getInstance();
        Leds leds = Leds.getInstance();
        addCommands(
                new ArmAxisControl(1, 0.02, 0)
                        .until(() -> gripper.getDistance() < ArmConstants.FEEDER_DISTANCE),
                new InstantCommand(() -> leds.setBlinkTime(LedConstants.FAST_BLINK_TIME)),
                new InstantCommand(() -> leds.setBlink(true, LedConstants.kGalaxiaBlue))
                        .until(() -> !gripper.isOpen())
        );
    }
}
