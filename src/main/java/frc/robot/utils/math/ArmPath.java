package frc.robot.utils.math;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Optional;
import java.util.Scanner;

public class ArmPath {
    private final InterpolatingDoubleMap shoulderMap;
    private final InterpolatingDoubleMap elbowMap;

    public ArmPath(File file) {
        shoulderMap = new InterpolatingDoubleMap();
        elbowMap = new InterpolatingDoubleMap();

        try {
            generatePath(file);
        } catch (Throwable t) {
            t.printStackTrace();
        }
    }

    public void generatePath(File file) throws FileNotFoundException {
        Scanner reader = new Scanner(file);
        double shoulderAngle, elbowAngle, time;

        while (reader.hasNextDouble()) {
            shoulderAngle = reader.nextDouble();
            elbowAngle = reader.nextDouble();
            time = reader.nextDouble();

            shoulderMap.put(shoulderAngle, time);
            elbowMap.put(elbowAngle, time);
        }
    }

    public Optional<InterpolatingDouble> getShoulderAngle(double t) {
        var shoulderAngle = shoulderMap.getInterpolated(new InterpolatingDouble(t));
        return Optional.ofNullable(shoulderAngle);
    }

    public Optional<InterpolatingDouble> getElbowAngle(double t) {
        var elbowAngle = elbowMap.getInterpolated(new InterpolatingDouble(t));
        return Optional.ofNullable(elbowAngle);
    }
}
