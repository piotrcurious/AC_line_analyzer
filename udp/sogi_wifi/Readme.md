SOGI version 
simple UDP multicast version - sender and receiver. 

the receiver can also be an AP.
using sender as AP causes ADC sampling problems (interference from AP)
but it is theoretically possible.

sender can log and visualize too, there is plenty of resources left. 

TODO : DC offsets are subtracted before sending - this is perhaps not really wise idea
and perhaps DC offset should be sent over network for full reconstruction. 

TODO : third channel for leakage current measurement (line vs PE) should be possible, though things get tight then . 
