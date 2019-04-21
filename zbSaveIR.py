#!/usr/bin/env python
# coding: latin-1

# Import library functions we need
import ZeroBorg
import time

# Settings for gathering results
filePath            = './zbIrButtonMap.py'
replaceExisting     = False
secondsToScan       = 1.0
waitBetweenReads    = 0.01

# Setup the ZeroBorg
ZB = ZeroBorg.ZeroBorg()
#ZB.i2cAddress = 0x40                   # Uncomment and change the value if you have changed the board address
ZB.init()
if not ZB.foundChip:
    boards = ZeroBorg.scan_for_zero_borg()
    if len(boards) == 0:
        print 'No ZeroBorg found, check you are attached :)'
    else:
        print 'No ZeroBorg at address %02X, but we did find boards:' % (ZB.i2cAddress)
        for board in boards:
            print '    %02X (%d)' % (board, board)
        print 'If you need to change the I²C address change the setup line so it is correct, e.g.'
        print 'ZB.i2cAddress = 0x%02X' % (boards[0])
    sys.exit()
#ZB.set_epo_ignore(True)                 # Uncomment to disable EPO latch, needed if you do not have a switch / jumper
ZB.reset_epo()

ZB.set_led_ir(True)
print ''
if replaceExisting:
    fOut = open(filePath, 'w')
else:
    fOut = open(filePath, 'a')
fOut.write('# Button map for IR remote using ZeroBorg\n')
fOut.write('# You can generate a new version of this script using zbSaveIr.py\n\n')
while True:
    print 'Button names can use letters, numbers and the _ symbol only'
    buttonName = raw_input('Hold the remote button down, type a name and press enter\n>')
    if len(buttonName) == 0:
        # No name, exit the loop
        break
    print 'Keep holding the button down...'
    # Start by reading any existing messages to clear the new message flag
    ZB.get_ir_message()
    startTime = time.time()
    messages = {}
    # Loop for the set timeout
    while (time.time() - startTime) < secondsToScan:
        # See if there is a new message
        if ZB.has_new_ir_message():
            # Store a count of each received message
            messageData = ZB.get_ir_message()
            if messages.has_key(messageData):
                messages[messageData] += 1
            else:
                messages[messageData] = 1
        # Wait a while before checking again
        time.sleep(waitBetweenReads)
    print 'You can release the button now'
    # Sort the results
    messageScores = []
    for key in messages.keys():
        messageScores.append([messages[key], key])
    messageScores.sort()
    messageScores.reverse()
    if len(messageScores) == 0:
        print 'No messages seen, try again.'
    else:
        # Display the results
        print '------'
        for result in messageScores:
            print '%s seen %d times' % (result[1], result[0])
        print '------'
        setting = 'IR_%s = "%s"\n' % (buttonName, messageScores[0][1])
        print 'Adding ' + setting
        fOut.write(setting)
ZB.set_led_ir(False)
fOut.close()
print 'Done'
