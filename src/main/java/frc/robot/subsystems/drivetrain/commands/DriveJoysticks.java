package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.math.AngleUtil;

public class DriveJoysticks extends HolonomicDrive {
    private final Joystick leftJoystick;

    public DriveJoysticks(Joystick leftJoystick, Joystick rightJoystick) {
        super(() -> -leftJoystick.getY(), () -> -leftJoystick.getX(), () -> -rightJoystick.getX(),
                rightJoystick::getTrigger, () -> rightJoystick.getRawButton(3), leftJoystick::getTrigger);
        this.leftJoystick = leftJoystick;
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = calculateVelocities();
        int pov = leftJoystick.getPOV();
        if (pov >= 0) {
            AngleUtil.Angle povObject = new AngleUtil.Angle(AngleUtil.UP_CLOCKWISE, pov);
            double diff = Math.toRadians(povObject.minus(Robot.gyroscope.getYawObject()));
            swerveDrive.drive(speeds, new Translation2d(
                    Math.cos(diff) * Constants.SwerveDrive.TORNADO_SPIN_DISTANCE,
                    Math.sin(diff) * Constants.SwerveDrive.TORNADO_SPIN_DISTANCE));
        } else {
            super.execute();
        }
    }
}
