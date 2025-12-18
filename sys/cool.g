G90
;M83
M140 S0            ; Turn off heated bed
M104 S0
M106 P2 S1.0
G1 X55 Y0 Z150 B0 C0 F2000
while {heat.heaters[0].current > 35}
    G1 C360 F2000
    G92 C0

M106 P2 S0.0


