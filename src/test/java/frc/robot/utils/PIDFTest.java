package frc.robot.utils;

import frc.robot.utils.controllers.DieterController;
import org.junit.Assert;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

import java.io.File;
import java.util.Scanner;

@RunWith(JUnit4.class)
public class PIDFTest {
    public static final double EPSILON = 1e-4;

    @Test
    public void pidfTest() {
        try {
            DieterController dieterController;
            Scanner in = new Scanner(new File("test-files/pidf.txt"));
            int numControllers = in.nextInt(), numTests;
            for (int i = 0; i < numControllers; i++) {
                dieterController = new DieterController(
                        in.nextDouble(),
                        in.nextDouble(),
                        in.nextDouble(),
                        in.nextDouble()
                );
                numTests = in.nextInt();
                for (int j = 0; j < numTests; j++) {
                    try {
                        Assert.assertEquals(
                                "PIDF test #" + (i + 1) + "\n",
                                dieterController.calculate(in.nextDouble(), in.nextDouble()),
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
}
