I made a new version of my SI4732/35_2.8_TFT_V3.0 sketch. In version 3.1 I first applied Jim Yasuda's beautiful Sprite keys. Well done Jim. Thank you. In order for everything to work flawlessly, the latest version of the TFT_eSPI library must be installed. https://github.com/Bodmer/TFT_eSPI . Don't forget to save the files Setup1_ILI9341 and User_Setup_Select from the TFT_eSPI lib. These files are overwritten when installing the new version of the library.

I also made a kind of suppression for the nasty chuffing when tuning in SSB with the 1kHz step. The AGC control causes this noise. It lasts about 350mSec, peaking in the first 200mSec. I mute the LF signal with an adjustable time by making the two transistors in the LF circuit conductive. If the time is too long, tuning becomes difficult. It then takes too long before the result of the frequency change becomes audible. 200mSec seems like a good compromise. This time can be set in line 115 of the V3.1 sketch:
#define MIN_ELAPSED_AudMut_TIME 200. 
If 0 is entered here, the function is disabled.


