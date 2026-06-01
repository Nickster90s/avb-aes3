#!/usr/bin/env bash
# Send 'a' to the FPGA console and capture the AAF/soft-ILA status incl. the
# new per-second rate line. Console is /dev/ttyACM0 @ 1 Mbaud, root:dialout
# (needs sudo; the 'lisp' user is not in dialout).
#
# Press twice ~1s apart while USB is streaming: the FIRST press seeds the rate
# window, the SECOND shows real /s rates. Decisive line:
#   rates(/s): strobe= usb_samp= (frames=) push= pop= first=  [host~46979]
#     strobe  = true NCO consumer demand (48000? 48007?)
#     frames  = usb_samp/8 = true USB producer frame rate (should ~= 46979)
#     first   > frames  =>  `first` glitching => phantom pushes (FIFO pins full)
#     pop vs strobe, and level min/max below, finish the picture.
DEV=/dev/ttyACM0
sudo stty -F $DEV 1000000 raw -echo
sudo bash -c "timeout ${1:-4} cat $DEV > /tmp/console_rates.log 2>&1 &
  sleep 0.5; printf 'a\r' > $DEV; sleep ${1:-4}"
sudo strings /tmp/console_rates.log | grep -E "rates\(/s\)|\[AAF\]|aaf_pkt\(gw\)|soft-ila|usb-fifo|usb-bridge" || sudo cat /tmp/console_rates.log
