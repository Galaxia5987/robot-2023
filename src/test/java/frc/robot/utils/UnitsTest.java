package frc.robot.utils;

import frc.robot.utils.units.Units;
import org.junit.Assert;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

import java.io.File;
import java.util.Scanner;

/*
Unit test class for the Units class.
Format for input is as such:
    For every function:
        1. Write the number of inputs.
        2. After the number of inputs write pairs of inputs and solution,
            divided by a space.
 */
@RunWith(JUnit4.class)
public class UnitsTest {
    public static final double EPSILON = 1e-4;

    UnitFunction[] unitFunctions = new UnitFunction[]{
            Units::rpmToRadsPerSec,
            Units::radsPerSecToRpm,
            Units::rpsToRpm,
            Units::rpmToRps,
            Units::radsPerSecToRps,
            Units::rpsToRadsPerSec,
            Units::inchesToMeters,
            Units::metersToInches,
            Units::feetToMeters,
            Units::metersToFeet
    };

    @Test
    public void unitsTest() {
        int numSolutions;
        try {
            Scanner in = new Scanner(new File("test-files/units.txt"));
            for (UnitFunction function : unitFunctions) {
                numSolutions = in.nextInt();
                for (int i = 0; i < numSolutions; i++) {
                    try {
                        Assert.assertEquals(
                                "Unit function " + function.toString(),
                                function.get(in.nextDouble()),
                                in.nextDouble(),
                                EPSILON
                        );
                    } catch (Throwable t) {
                        t.printStackTrace();
                    }
                }
            }
        } catch (Throwable t) {
            t.printStackTrace();
        }

    }

    public interface UnitFunction {
        double get(double in);
    }
}
