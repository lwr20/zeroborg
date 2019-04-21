#!/usr/bin/env python
# coding: latin-1

# Import library functions we need
import ZeroBorg
import time

# Setup the ZeroBorg
ZB = ZeroBorg.ZeroBorg()
#ZB.i2c_address = 0x40                   # Uncomment and change the value if you have changed the board address
ZB.init()
if not ZB.foundChip:
    boards = ZeroBorg.scan_for_zero_borg()
    if len(boards) == 0:
        print 'No ZeroBorg found, check you are attached :)'
    else:
        print 'No ZeroBorg at address %02X, but we did find boards:' % (ZB.i2c_address)
        for board in boards:
            print '    %02X (%d)' % (board, board)
        print 'If you need to change the I�C address change the setup line so it is correct, e.g.'
        print 'ZB.i2c_address = 0x%02X' % (boards[0])
    sys.exit()
#ZB.set_epo_ignore(True)                 # Uncomment to disable EPO latch, needed if you do not have a switch / jumper
ZB.reset_epo()

try:
    # Loop forever
    print 'Press CTRL+C to quit'
    while True:
        # Read the new volatges
        analog1 = ZB.get_analog1()
        analog2 = ZB.get_analog2()
        # Display the readings
        print '#1: %01.2f V     #2: %01.2f V' % (analog1, analog2)
        # Wait a while before checking again
        time.sleep(0.1)
except KeyboardInterrupt:
    # CTRL+C exit
    print 'Terminated'

