package frc.robot.commandgroups;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.utils.controllers.PIDFController;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class AdjustToAprilTag extends CommandBase {
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final Limelight limelight = Limelight.getInstance();
    private final Gyroscope gyroscope = Gyroscope.getInstance();

    private final PIDFController yController = new PIDFController(10, 0, 0, 0.5);
    private final PIDFController thetaController = new PIDFController(16.0, 0, 0, 0);

    private final DoubleSupplier xSupplier;

    private double lastVy = 0;

    public AdjustToAprilTag(DoubleSupplier xSupplier){
        this.xSupplier = xSupplier;
    }

    @Override
    public void execute() {
        var botPose = limelight.getBotPose();
        double omegaVelocity = 0, yVelocity = lastVy;

        if (botPose.isPresent()) {
            yVelocity = -yController.calculate(botPose.get().getY(), 0.1);
            omegaVelocity = thetaController.calculate(botPose.get().getRotation().getRadians(), 0);
            Logger.getInstance().recordOutput("BotPose", botPose.get());
        }

        swerveDrive.drive(
                new DriveSignal(
                        0,
                        yVelocity,
                        omegaVelocity,
                        new Translation2d(),
                        true
                )
        );

        lastVy = yVelocity;
    }
}
