SOGI version 
simple UDP multicast version - sender and receiver. 

the receiver can also be an AP.
using sender as AP causes ADC sampling problems (interference from AP)
but it is theoretically possible.

sender can log and visualize too, there is plenty of resources left. 

TODO : DC offsets are subtracted before sending - this is perhaps not really wise idea
and perhaps DC offset should be sent over network for full reconstruction. 

TODO : third channel for leakage current measurement (line vs PE) should be possible, though things get tight then . 

TODO: the "double" version tries doing math in doubles but it is too slow. it needs just a little optimize as it was converted blindly, some operations do not really benefit from double and the numerical errors are actually coming from clock jitter and overlapping quantization of clocks,samples and other effects rather than integration. It gets about one order of magnitude better SNR ar current state. 

bottom line - 
if you think you need it, think twice - consider using BLE wireless audio module instead (stereo, built-in jitter compensation, 196khz sample rate, 16bit and sometimes even 24bit, compression, much better spectral efficiency and range)
