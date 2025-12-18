; repeatwitheject.g
; Usage:
;   M98 P"repeatwitheject.g" F"eject.g" N2

; --- Validate parameters ---
if !exists(param.F)
    abort "repeatwitheject.g error: Missing parameter F (filename)."

if !exists(param.N)
    abort "repeatwitheject.g error: Missing parameter N (repeat count)."

if param.N <= 0
    abort "repeatwitheject.g error: N must be > 0."

echo "Repeating file '" ^ param.F ^ "' for " ^ param.N ^ " iterations."

; --- Initialize counter ---
var idx = 0

; --- Loop N times using WHILE (fully supported) ---
while var.idx < param.N
    set var.idx = var.idx + 1
    echo "Iteration " ^ var.idx ^ " of " ^ param.N
    M98 P{param.F}
    M98 P"cool.g"
    M98 P"eject.g"
