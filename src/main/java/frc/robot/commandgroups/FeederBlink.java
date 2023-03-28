package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.leds.LedConstants;
import frc.robot.subsystems.leds.Leds;

public class FeederBlink extends SequentialCommandGroup {

    public FeederBlink(){
        Gripper gripper = Gripper.getInstance();
        Leds leds = Leds.getInstance();
        addRequirements(gripper, leds);
        addCommands(
                new ConditionalCommand(
                        new InstantCommand(() ->leds.setBlink(true, LedConstants.kGalaxiaBlue, LedConstants.FAST_BLINK_TIME)),
                        new InstantCommand(() -> leds.setBlink(false)),
                        () -> gripper.getDistance() < ArmConstants.FEEDER_DISTANCE && gripper.isOpen()
                )
        );
    }
}
