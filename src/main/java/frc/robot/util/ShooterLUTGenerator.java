package frc.robot.util;

import frc.lib.firecontrol.ProjectileSimulator;
import frc.lib.firecontrol.ShotLUT;

public class ShooterLUTGenerator {

    /**
     * Generates a lookup table for the Shoot-on-the-Move SOTM fire control solver.
     */
    public static ShotLUT generateLUT() {
        // Initialize the physical simulation values based on physical properties
        // M to inch conversion factor roughly 39.37
        ProjectileSimulator.SimParameters params = new ProjectileSimulator.SimParameters(
            0.215,   // ball mass kg
            0.1501,  // ball diameter m (~6 in)
            0.47,    // drag coeff (smooth sphere)
            0.2,     // Magnus coeff
            1.225,   // air density
            0.5588,    // exit height (m), ~22 in.
            0.0762,  // flywheel diameter (m) ~ 3 in
            1.83,    // target height (m) approx speaker/hub center
            0.6,     // slip factor
            15.0,    // launch angle from horizontal (baseline)
            0.001,   // sim timestep
            1500, 6000, 25, 5.0  // search range, iterations, max time
        );

        ProjectileSimulator sim = new ProjectileSimulator(params);
        // sweep angles from 15 to 30 degrees in 1-degree steps
        ShotLUT generatedLUT = sim.generateVariableAngleShotLUT(0.0, 15.0, 0.5);

        System.out.println("====== GENERATED SHOOTER LUT ======");
        for (double d = 0.5; d <= 5.0; d += 0.1) {
            double distance = Math.round(d * 10.0) / 10.0;
            System.out.printf("Dist: %.2fm -> %.0f RPM / Angle: %.1f deg (%.3fs)%n", 
                distance, generatedLUT.getRPM(distance), generatedLUT.getAngle(distance), generatedLUT.getTOF(distance));
        }
        System.out.println("===================================");

        return generatedLUT;
    }
}
