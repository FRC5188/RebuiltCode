package frc.lib.Rebuilt2026;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;


public class OrchestraMode {

    Orchestra orchestra = new Orchestra();

    public OrchestraMode(TalonFX... orchestraMotors) {

        for (int i = 0; i < orchestraMotors.length; i++) {
            orchestra.addInstrument(orchestraMotors[i]);
        }

    }
    
    /** Loads audio file for orchestraToPlay
     * Example call orchestraLoadFile("sound/reaper.chrp")
     *  @param filePath Chirp file is inside your "src/main/deploy" directory
    */
    public void orchestraLoadFile(String filePath) {
        orchestra.loadMusic(filePath);
    }

    /** Disables the orchestra so the motor can preform normal motor control functions */
    public void orchestraDisable() {
        orchestra.stop();
    }

    /** Stop the music from playing does not allow motor to preform normal motor control functions */
    public void orchestraPause() {
        orchestra.pause();
    }

    /** If orchestra is pause resumes playing*/
    public void orchestraPlay() {
        orchestra.play();
    }

    /**Closes Orchestra object will not be able to use orchestra till created agian */
    public void orchestraClose() {
        orchestra.close();
    }
}