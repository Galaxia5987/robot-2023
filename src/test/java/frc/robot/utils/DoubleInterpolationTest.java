package frc.robot.utils;

import frc.robot.utils.math.InterpolatingDouble;
import frc.robot.utils.math.InterpolatingDoubleMap;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

import java.io.File;
import java.util.Arrays;
import java.util.Scanner;

/*
Unit test class for the InterpolatingDoubleMap class.
Format for input is as such:
    1. Input the number of points to interpolate between.
    2. After the number of inputs write pairs of points,
        with the axis values divided by a space.
    3. Input the number of solutions to test.
    4. Write pairs of inputs and solutions,
        also separated by a space.
 */
@RunWith(JUnit4.class)
public class DoubleInterpolationTest {
    public static final double EPSILON = 1e-4;

    private final InterpolatingDoubleMap map;
    private double[][] solutions = null;

    public DoubleInterpolationTest() {
        map = new InterpolatingDoubleMap();
    }

    @Before
    public void input() {
        try {
            int numPoints, numSolutions;
            File input = new File("test-files/double-interpolation.txt");
            Scanner in = new Scanner(input);
            numPoints = in.nextInt();
            for (int i = 0; i < numPoints; i++) {
                map.put(in.nextDouble(), in.nextDouble());
            }

            numSolutions = in.nextInt();
            solutions = new double[numSolutions][2];
            for (int i = 0; i < numSolutions; i++) {
                solutions[i][0] = in.nextDouble();
                solutions[i][1] = in.nextDouble();
            }
        } catch (Throwable t) {
            t.printStackTrace();
        }
    }

    @Test
    public void run() {
        for (double[] solution : solutions) {
            try {
                Assert.assertEquals("Point" + Arrays.toString(solution),
                        map.getInterpolated(new InterpolatingDouble(solution[0])).value,
                        solution[1],
                        EPSILON);
            } catch (Throwable t) {
                t.printStackTrace();
            }
        }
    }
}
