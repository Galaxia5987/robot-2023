package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class MeasurePositionsToCSV extends CommandBase {
    private final Arm arm = Arm.getInstance();
    private final File file = new File("/home/lvuser/arm_path.csv");
    private FileWriter writer;

    private String output = "";
    private final Timer timer = new Timer();

    public MeasurePositionsToCSV() {
    }

    @Override
    public void initialize() {
        timer.start();
        timer.reset();

        try {
            file.createNewFile();
            System.out.println(file.exists());
            writer = new FileWriter(file);
        } catch (Throwable t) {
            t.printStackTrace();
        }
    }

    @Override
    public void execute() {
        double shoulderAngle = arm.getShoulderJointAngle().getDegrees();
        double elbowAngle = arm.getElbowJointAngle().getDegrees();
        output += shoulderAngle + "," + elbowAngle + "," + timer.get() + "\n";
    }

    @Override
    public void end(boolean interrupted) {
        try {
            writer.write(output);
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
