; eject_elephant.g

G90
;M83
;M140 S0            ; Turn off heated bed
;M104 S0
;while {heat.heaters[0].current > 35}
;    G4 S1               ; wait 1 second

; Step 1: Move to safe Z, then to X end
;G1 X65 F3000;
G1 Z120 F5000
G1 B-44 C0 F5000
;G1 Z70 F1000
G1 X60 Y90 Z70 F5000



;G1 Z70 F6000       ; move to Z = 55mm
;G1 X98 F6000    ; go to X endstop
;G1 Z0 F6000        ; drop to Z = 0 (bed level)

;G1 Z100 F6000        ; lift the tool 40mm up from the bed


; Step 2:     Rotate head 45° CCW

;G1 B-44 F1000;   rotate head CCW

;G1 X98 F6000 

;G1 Z20 F6000;
; -------------------------------
; Step 3: Fast X sweep for part ejection
; -------------------------------

; ✅ FIX HERE: use axes[0] for X, not axes[1] (which is Y)
;var oldXAccel = move.axes[0].acceleration
;M201 X6000                     ; temporary high accel for snappy swipe

G1 X-80 F3000                ; full-speed sweep across X range
G1 X70 F20000
G1 Y30 F10000
G1 X-80 F3000                ; full-speed sweep across X range
G1 X70 F20000
G1 Y-30 F10000
G1 X-80 F3000                ; full-speed sweep across X range
G1 X0 Y60 Z120 B0 C90 F5000
G92 X60 Y0 C0

;M201 X{var.oldXAccel}          ; restore original accel
