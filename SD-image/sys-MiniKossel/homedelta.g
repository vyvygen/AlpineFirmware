; Homing file for RepRapFirmware on Mini Kossel
G91							; use relative positioning
G1 S1 X250 Y250 Z250 F5000	; move all carriages up 250mm, stopping at the endstops
G1 S2 X-5 Y-5 Z-5 F1000		; move all towers down 5mm
G1 S1 X8 Y8 Z8 F1000		; move towers up 8mm, stopping at the endstops
G90							; back to absolute positioning
