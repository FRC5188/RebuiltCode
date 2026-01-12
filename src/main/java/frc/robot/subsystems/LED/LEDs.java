// package frc.robot.LEDs;

// import com.ctre.phoenix6.configs.CANdleConfiguration;
// import com.ctre.phoenix6.hardware.CANdle;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// import com.ctre.phoenix.led.Animation;

// public class LEDs extends SubsystemBase {

    // List of LED color options
  //  public enum LEDColors {

        // DONE #1: Add a color of your own to the list. When you're finished, replace 'TOD0' with
        // "DONE".
    //    Fire,
      //  Off
  //  }

    // Eight LEDs built in to CANdle. If LED strips are added, add (8 + LED_strip_length).
    // private final int LEDCount = 58;

//    public CANdle _candle = new CANdle(Constants.CANDLE_ID, "rio");

  //  public LEDColors _currentColor;

    // public LEDs() {

      //  System.out.println("White LEDs");
        // _currentColor = LEDColors.Fire;

//        CANdleConfiguration configAll = new CANdleConfiguration();
  //      configAll.FutureProofConfigs = true;
    //    configAll.FutureProofConfigs = false; // Don't turn off leds when we lose comms. let us
    // decide
      //  configAll.stripType = LEDStripType.RGB; // Normal RBG strips
        // configAll.brightnessScalar = 0.5; // Half brightness
//        configAll.vBatOutputMode = VBatOutputMode.Modulated;
  //      _candle.configAllSettings(configAll, 100);

    // }

    /**
     * Changes the color of the LEDs. Called by setLEDMode method.
     *
     * @param color - Input enumeration name from LEDColors (ex: White)
     */

    // public void setColor(LEDColors color) {

    //    switch(color) {
      //      _candle.
        //    case Fire:
          //          m_candle.setControl(
            //            new FireAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
              //              .withDirection(AnimationDirectionValue.Backward)
                //            .withCooling(0.4)
                  //          .withSparking(0.5)
                    // );
                    // break;

//            case Off:
  //              _candle.setLEDs(0, 0, 0);
                // _candle.configBrightnessScalar(0.0, 100);
    //            this._currentColor = LEDColors.Off;
      //          break;

        //    default:
          //      _candle.setLEDs(255, 255, 255);
            //    this._currentColor = LEDColors.White;
    //    }
   // }

    /**
     * @return Double - temperature in Celsius
     */

  //  public Double getLEDTemperature() {
//        return this._candle.getTemperature();
    // }

    /**
     * If LEDs are greater that 80 degrees Celsius, turn them off. Prevents vision (which is
     * connected) from failing. Called by LEDDefault command.
     *
     * @param temperature - Input Double for temperature (ex: currentTemperature)
     */

  //  public void adjustLEDTemperature(Double temperature) {
//        if (temperature >= 80.0) {
        //    this._candle.setLEDs(0, 0, 0);
      //      this._candle.configBrightnessScalar(0.0);
    //    }
  //  }
// }
