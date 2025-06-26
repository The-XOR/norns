#!/bin/bash

aconnect -x

# Get aconnect output
ACONNECT_OUTPUT=$(aconnect -l)

# Find ttymidi client number (could be named "ttymidi", "laMIDI", etc.)
TTYMIDI_CLIENT=$(echo "$ACONNECT_OUTPUT" | grep -i -E "(ttymidi|lamidi)" | grep "client" | sed 's/client \([0-9]*\):.*/\1/')

# Find first Virtual Raw MIDI client number
VIRTUAL_CLIENT=$(echo "$ACONNECT_OUTPUT" | grep "Virtual Raw MIDI" | head -1 | sed 's/client \([0-9]*\):.*/\1/')

aconnect $TTYMIDI_CLIENT:0 $VIRTUAL_CLIENT:0
aconnect $VIRTUAL_CLIENT:0 $TTYMIDI_CLIENT:1;
