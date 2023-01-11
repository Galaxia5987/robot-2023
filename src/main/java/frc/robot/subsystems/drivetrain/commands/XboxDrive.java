package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.XboxController;

public class XboxDrive extends HolonomicDrive {

    public XboxDrive(XboxController xboxController) {
        super(() -> -xboxController.getLeftY(), () -> -xboxController.getLeftX(), () -> -xboxController.getRightX());
    }
}
