Basandosi sul file "dts list", il display ST7735r viene attivato col comando:

dtoverlay=adafruit18,reset_pin=6,dc_pin=5,rotate=270

NOTA:
il piedino DC corrisponde a RS nel pdf allegato.
L'utilizzo del termine "PIN" è improprio, in realtà è il numero di GPIO.

