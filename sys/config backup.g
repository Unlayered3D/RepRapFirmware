; Configuration file for RepRapFirmware on Duet 3 Mini 5+ WiFi
; executed by the firmware on start-up
;
; Written by Alex Stedman 
;
; 5 Axis firmware for a 5 axis printer (RPPPR) 
; Axes are CYZXB, but X, Y, and Z are interchangeable because they are all prismatic 
; This means that we will address in convention CXYZB
; C and Y, and B and X are coupled by a differential, thus we will name it CoreXBYC
; THis is similar to how CoreXYUV is named 
;
; General
G90 ; absolute coordinates
M83 ; relative extruder moves
M550 P"Unlayered 5X Printer" ; set hostname

; Accessories
M575 P1 S0 B57600 ; configure PanelDue support

; Network
M552 S1 ; configure WiFi adapter
M586 P0 S1 ; configure HTTP

; Smart Drivers
M569 P0.0 S1 D3 V100 ; driver 0.0 goes forward (XB axis)
M569 P0.1 S0 D3 V100 ; driver 0.1 goes backwards (YC axis)
M569 P0.2 S1 D3 V100 ; driver 0.2 goes forwards (Z axis)
M569 P0.3 S0 D3 V100 ; driver 0.3 goes backwards (XB axis)
M569 P0.4 S1 D3 V100 ; driver 0.4 goes forwards (YC axis)
M569 P0.5 S1 D3 V2000 ; driver 0.5 goes forwards (extruder 0)

; Motor Idle Current Reduction
M906 I30 ; set motor current idle factor
M84 S30 ; set motor current idle timeout

; Axes
M584 X0.0 Y0.1 Z0.2 ; set axis mapping for XYZ
M584 B0.3; end of mappings for hotend
M584 C0.4; start of mappings for bed


M350 X16 Y16 Z16 I1 ; also
M350 B16 I1 ; also
M350 C16 I1 ; configure microstepping with interpolation

M906 X800 Y800 Z800 ; set axis driver currents
M906 B800 ; current
M906 C800 ; current

M92 X80 Y80 Z400 ; configure steps per mm
M92 B80 ;26.66667 ; 80 belt mm * 60*2/360 = 80 * 1/3 = 26.66667
M92 C80 ;48.88889 ; 80 belt mm * 110*2/360 = 80 * 0.6111 = 48.88889

M208 X-98:98 Y-98:98 Z0:130 B-45:135 C-3600:3600; set minimum and maximum axis limits

M566 X900 Y900 Z300 ; set maximum instantaneous speed changes (mm/min)
M566 B900 ; (deg/min)
M566 C900 ; (deg/min)

M203 X6000 Y6000 Z1000 ; set maximum speeds (mm/min)
M203 B6000; (deg/min)
M203 C6000 ; (deg/min)

M201 X1000 Y1000 Z300 ; set accelerations (mm/s^2)
M201 B1000 ; rotational acceleration (deg/s^2)
M201 C1000 ; rotational acceleration (deg/s^2)

; Extruders
M584 E0.5 ; set extruder mapping
M350 E16 I1 ; configure microstepping with interpolation
M906 E1000 ; set extruder driver currents
M92 E420 ; configure steps per mm
M566 E120 ; set maximum instantaneous speed changes (mm/min)
M203 E3600 ; set maximum speeds (mm/min)
M201 E250 ; set accelerations (mm/s^2)

; Kinematics
M669 K16; configure 5 axis kinematics
; 15 and 17 are other modes

; Probes
M558 K0 P8 C"io6.in" H5 F120 T6000 ; configure unfiltered digital probe via slot #0
G31 P500 X0 Y0 Z0.7                ; set Z probe trigger value, offset and trigger height

; Endstops
M574 X0 ; configure X axis endstop
M574 Y1 S3; configure Y axis endstop
M574 Z1 S2 ; configure Z axis endstop

; Sensors
M308 S0 P"temp0" Y"thermistor" A"Heated Bed" T100000 B4725 C7.06e-8 ; configure sensor #0
M308 S1 P"temp1" Y"thermistor" A"Nozzle" T100000 B4725 C7.06e-8 ; configure sensor #1

; Heaters
M950 H0 C"out0" T0 ; create heater #0
M143 H0 P0 T0 C0 S100 A0 ; configure heater monitor #0 for heater #0
M307 H0 R2.43 D5.5 E1.35 K0.56 B1 ; configure model of heater #0
M950 H1 C"out1" T1 ; create heater #1
M143 H1 P0 T1 C0 S300 A0 ; configure heater monitor #0 for heater #1
M307 H1 R2.43 D5.5 E1.35 K0.56 B0 ; configure model of heater #1

; Heated beds
M140 P0 H0 ; configure heated bed #0

; Fans
M950 F0 C"out6" ; create fan #0
M106 P0 S0 L0 X1 B0.1 ; configure fan #0
M950 F1 C"out5" ; create fan #1
M106 P1 S0 B0.1 H1 T45 ; configure fan #1

; Tools
M563 P0 D0 H1 F0 ; create tool #0
M568 P0 R0 S0 ; set initial tool #0 active and standby temperatures to 0C

; Miscellaneous
T0 ; select first tool
M552 s3;


; Configuration file for RepRapFirmware on Duet 3 Mini 5+ WiFi
; executed by the firmware on start-up
;
; Written by Alex Stedman 
;
; 5 Axis firmware for a 5 axis printer (RPPPR) 
; Axes are CYZXB, but X, Y, and Z are interchangeable because they are all prismatic 
; This means that we will address in convention CXYZB
; C and Y, and B and X are coupled by a differential, thus we will name it CoreXBYC
; THis is similar to how CoreXYUV is named 
;
; General
G90 ; absolute coordinates
M83 ; relative extruder moves
M550 P"Unlayered 5X Printer" ; set hostname

; Accessories
M575 P1 S0 B57600 ; configure PanelDue support

; Network
M552 S1 ; configure WiFi adapter
M586 P0 S1 ; configure HTTP

; Smart Drivers
M569 P0.0 S0 D3 V100 ; driver 0.0 goes backwards (XB axis)
M569 P0.1 S0 D3 V100 ; driver 0.1 goes backwards (YC axis)
M569 P0.2 S1 D3 V100 ; driver 0.2 goes forwards (Z axis)
M569 P0.3 S1 D3 V100 ; driver 0.3 goes forwards (XB axis)
M569 P0.4 S1 D3 V100 ; driver 0.4 goes forwards (YC axis)
M569 P0.5 S1 D3 V2000 ; driver 0.5 goes forwards (extruder 0)

; Motor Idle Current Reduction
M906 I30 ; set motor current idle factor
M84 S30 ; set motor current idle timeout

; Axes
M584 X0.0 Y0.1 Z0.2 ; set axis mapping for XYZ
M584 B0.3; end of mappings for hotend
M584 C0.4; start of mappings for bed


M350 X16 Y16 Z16 I1 ; also
M350 B16 I1 ; also
M350 C16 I1 ; configure microstepping with interpolation

M906 X800 Y800 Z800 ; set axis driver currents
M906 B800 ; current
M906 C800 ; current

M92 X80 Y80 Z400 ; configure steps per mm
M92 B80 ;26.66667 ; 80 belt mm * 60*2/360 = 80 * 1/3 = 26.66667
M92 C80 ;48.88889 ; 80 belt mm * 110*2/360 = 80 * 0.6111 = 48.88889

M208 X-98:98 Y-98:98 Z0:130 B-135:45 C-3600:3600; set minimum and maximum axis limits

M566 X900 Y900 Z100 ; set maximum instantaneous speed changes (mm/min)
M566 B900 ; (deg/min)
M566 C900 ; (deg/min)

M203 X6000 Y6000 Z400 ; set maximum speeds (mm/min)
M203 B6000; (deg/min)
M203 C6000 ; (deg/min)

M201 X1000 Y1000 Z300 ; set accelerations (mm/s^2)
M201 B1000 ; rotational acceleration (deg/s^2)
M201 C1000 ; rotational acceleration (deg/s^2)

; Extruders
M584 E0.5 ; set extruder mapping
M350 E16 I1 ; configure microstepping with interpolation
M906 E1000 ; set extruder driver currents
M92 E420 ; configure steps per mm
M566 E120 ; set maximum instantaneous speed changes (mm/min)
M203 E3600 ; set maximum speeds (mm/min)
M201 E250 ; set accelerations (mm/s^2)

; Kinematics
M669 K16; configure 5 axis kinematics
; 15 and 17 are other modes

; Probes
M558 K0 P8 C"io6.in" H5 F120 T6000 ; configure unfiltered digital probe via slot #0
G31 P500 X0 Y0 Z0.7                ; set Z probe trigger value, offset and trigger height

; Endstops
M574 X0 ; configure X axis endstop
M574 Y1 S3; configure Y axis endstop
M574 Z1 S2 ; configure Z axis endstop

; Sensors
M308 S0 P"temp0" Y"thermistor" A"Heated Bed" T100000 B4725 C7.06e-8 ; configure sensor #0
M308 S1 P"temp1" Y"thermistor" A"Nozzle" T100000 B4725 C7.06e-8 ; configure sensor #1

; Heaters
M950 H0 C"out0" T0 ; create heater #0
M143 H0 P0 T0 C0 S100 A0 ; configure heater monitor #0 for heater #0
M307 H0 R2.43 D5.5 E1.35 K0.56 B1 ; configure model of heater #0
M950 H1 C"out1" T1 ; create heater #1
M143 H1 P0 T1 C0 S300 A0 ; configure heater monitor #0 for heater #1
M307 H1 R2.43 D5.5 E1.35 K0.56 B0 ; configure model of heater #1

; Heated beds
M140 P0 H0 ; configure heated bed #0

; Fans
M950 F0 C"out6" ; create fan #0
M106 P0 S0 L0 X1 B0.1 ; configure fan #0
M950 F1 C"out5" ; create fan #1
M106 P1 S0 B0.1 H1 T45 ; configure fan #1

; Tools
M563 P0 D0 H1 F0 ; create tool #0
M568 P0 R0 S0 ; set initial tool #0 active and standby temperatures to 0C

; Miscellaneous
T0 ; select first tool
M552 s3;


