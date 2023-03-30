package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.intake.ProximitySensor;
import frc.robot.subsystems.leds.LedConstants;
import frc.robot.subsystems.leds.Leds;


public class ProximitySensorDefualtCommand extends CommandBase {
    private final ProximitySensor proximitySensor = ProximitySensor.getInstance();
    private final Leds leds = Leds.getInstance();
    private final Gripper gripper = Gripper.getInstance();

    private boolean ProximitySensorState = proximitySensor.isBeamBlocked();

    public ProximitySensorDefualtCommand() {
        this.ProximitySensorState = proximitySensor.isBeamBlocked();
        addRequirements(proximitySensor, leds);
    }

    @Override
    public void execute() {
        if (proximitySensor.isBeamBlocked() || (gripper.getDistance() < ArmConstants.FEEDER_DISTANCE && gripper.isOpen() && gripper.getDistance() > ArmConstants.FEEDER_MINIMUM_DISTANCE))
            leds.setBlink(true, LedConstants.kGalaxiaBlue, LedConstants.FAST_BLINK_TIME);
        else leds.setBlink(false);
    }

}
