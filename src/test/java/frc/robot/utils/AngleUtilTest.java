package frc.robot.utils;

import frc.robot.utils.math.AngleUtil;
import org.junit.Assert;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

import java.io.File;
import java.util.Scanner;

@RunWith(JUnit4.class)
public class AngleUtilTest {
    public static final double EPSILON = 1e-4;

    @Test
    public void testAngleUtil() {
        try {
            Scanner in = new Scanner(new File("test-files/angle-util.txt"));
            int numAbsoluteTests = in.nextInt();
            for (int i = 0; i < numAbsoluteTests; i++) {
                try {
                    AngleUtil.Angle angle = inputAngle(in);
                    Assert.assertEquals(
                            "Absolute test\n" + angle.toString(),
                            in.nextDouble(),
                            angle.getAbsoluteAngle(),
                            EPSILON
                    );
                } catch (Throwable t) {
                    t.printStackTrace();
                }
            }
        } catch (Throwable t) {
            t.printStackTrace();
        }
    }

    public AngleUtil.Angle inputAngle(Scanner in) {
        double zeroAngle = in.nextDouble();
        boolean clockwise = in.nextInt() != 0;
        double angle = in.nextDouble();
        return new AngleUtil.Angle(AngleUtil.CoordinateSystem.of(zeroAngle, clockwise), angle);
    }
}
