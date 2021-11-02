I made a new version for my TFT SI473X radio. V3.3 is now available on Github.

First of all, a 100 and 10 Hz digit has been added. The radio is now tunable in SSB in steps of 1 Khz, 100 Hz and 10 Hz. 
The desired digit can be selected on the TFT screen by pressing it. The selected digit is underlined. 
Using the BFO's plus or minus 16 KHz tunable oscillator for tuning, the annoying puffing during a 1 KHz step has been reduced to once every 16 KHz.
A reduction of almost 94%. Tuning now takes place in almost total silence.
To improve the SSB reception, the crystal can be replaced in the program by an SI5351 clock generator. See the schematic on Github for this. 

I also use the clock generator for the BFO. The use of the BFO is almost no longer necessary after it has been set correctly. 
Only in the 80 meter band between 3700 and 3800 KHz are a few strange jumps. This occurs in all my prototypes with a SI4735 or SI4732. 
The clock generator can be calibrated using its 10 MHz signal. It is also possible to calibrate the generator in the radio. 
For this you need an SSB station on a known frequency and modulation. After tuning, the found setting can be saved by pressing the encoder button. 
If the BFO is set accurately, an accuracy of better than 0.1 Hz can be achieved. The clock generator set frequency is now 32768 Hz plus or minus a few tenths of Hz for the BFO. 
As reference station I use Shannon Volmet in Ireland on 5505 KHz USB. 
Stability is excellent at 32KHz 15 min after a cold start. If you put the radio in the sun, a certain gradient occurs. But for our application it is negligible.

Furthermore, the mode CW has been added. This is USB – 700Hz. 
This has the advantage that with a received tone of 700 Hz in CW the display indicates the original frequency of the station.

On the first screen, the volume can be adjusted by pressing the encoder key first. 
This can still be done with the VOL button. The volume can now be adjusted by turning the rotary encoder. When the button is pressed again, the regulation is switched off.

Several memory functions have been improved. In this way a changed of the modulation in a band remains active until you change it yourself. This does not apply to CW.

It is now also possible to control the screen lighting. This can be done in the second screen with the DISPL key. 
After pressing, the light intensity can be adjusted using the rotary encoder. 
The volume scale now indicates the setting. You can't turn off the screen completely, then you can't find the button to turn it back on again. 
You can switch off the control again with the rotary switch or with the DISP button. Setting is saved in memory.

The library for the SI5351 is under V3.3 as SI5351_wire.zip

In line 86 and 87 in the program you can select the crystal or the clock generator.
