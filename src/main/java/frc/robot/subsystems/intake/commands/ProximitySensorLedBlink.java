package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.ProximitySensor;
import frc.robot.subsystems.leds.Leds;


public class ProximitySensorLedBlink extends CommandBase {
    private final ProximitySensor proximitySensor = ProximitySensor.getInstance();
    private final Leds leds = Leds.getInstance();

    private boolean ProximitySensorState = proximitySensor.isBeamBlocked();

    public ProximitySensorLedBlink() {
        this.ProximitySensorState = proximitySensor.isBeamBlocked();
        addRequirements(proximitySensor);
    }

    @Override
    public void execute() {
        leds.setBlink(true);
    }

    @Override
    public void end(boolean interrupted) {
        leds.setBlink(false);
    }

    @Override
    public boolean isFinished() {
        return !proximitySensor.isBeamBlocked();
    }
}
