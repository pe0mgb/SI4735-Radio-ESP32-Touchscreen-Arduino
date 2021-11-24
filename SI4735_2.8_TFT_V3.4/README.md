In my version 3.3, I made choosing between the crystal and the clock generator unnecessarily complex. This was very confusing for some people. This has been resolved in the new version 3.4. To use the crystal, as in my previous designs or for the ATS-25, you don't have to change anything anymore. 
The use of the clock generator SI5351 must be indicated in the sketch.

For crystal (default) : 
- Line 82 #define IhaveCrystal.
- Line 83 // #define IhaveSI5351.

For si5351 :
- Line 82 //#define IhaveCrystal
- Line 83 #define IhaveSI5351

In the ATS-25 the transistor in the Vcc to the Backlight Led of the TFT is missing. The Led is directly connected to the Vcc. This makes it impossible to switch the Backlight on or off. This also makes dimming impossible. Furthermore, two transistors are missing in the LF circuit. This makes it impossible to suppress the loud pop that occurs when switching to different modulations.
The main change in V 3.4, when used with the crystal, is to store in the EEprom the BFO value for each band. This makes it less necessary to change the BFO. Furthermore, a few minor bugs have been fixed.

