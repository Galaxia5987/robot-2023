package frc.robot.subsystems.leds.command;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.leds.LedConstants;
import frc.robot.subsystems.leds.Leds;


public class Blink extends CommandBase {
    Leds leds = Leds.getInstance();
    Gripper gripper = Gripper.getInstance();
    Color color;

    public Blink (Color color){
        this.color = color;
    }

    @Override
    public void initialize() {
        leds.setBlinkTime(LedConstants.FAST_BLINK_TIME);
        System.out.println("I work");
    }

    @Override
    public void execute() {
        leds.setBlink(true, color);
        leds.setColor(color);
    }

    @Override
    public void end(boolean interrupted) {
        leds.setBlink(false);
        leds.setColor(LedConstants.kPurple);
    }

    @Override
    public boolean isFinished() {
        return !gripper.isOpen();
    }
}
