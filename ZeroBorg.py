#!/usr/bin/env python
# coding: latin-1
"""
This module is designed to communicate with the ZeroBorg

Use by creating an instance of the class, call the Init function, then command as desired, e.g.
import ZeroBorg
ZB = ZeroBorg.ZeroBorg()
ZB.Init()
# User code here, use ZB to control the board

Multiple boards can be used when configured with different I²C addresses by creating multiple instances, e.g.
import ZeroBorg
ZB1 = ZeroBorg.ZeroBorg()
ZB2 = ZeroBorg.ZeroBorg()
ZB1.i2c_address = 0x44
ZB2.i2c_address = 0x45
ZB1.Init()
ZB2.Init()
# User code here, use ZB1 and ZB2 to control each board separately

For explanations of the functions available call the help function, e.g.
import ZeroBorg
ZB = ZeroBorg.ZeroBorg()
ZB.help()
See the website at www.piborg.org/zeroborg for more details
"""

# Import the libraries we need
import io
import fcntl
import types
import time

# Constant values
I2C_SLAVE               = 0x0703
PWM_MAX                 = 255
I2C_NORM_LEN            = 4
I2C_LONG_LEN            = 24

I2C_ID_ZEROBORG         = 0x40

COMMAND_SET_LED         = 1     # Set the LED status
COMMAND_GET_LED         = 2     # Get the LED status
COMMAND_SET_A_FWD       = 3     # Set motor 1 PWM rate in a forwards direction
COMMAND_SET_A_REV       = 4     # Set motor 1 PWM rate in a reverse direction
COMMAND_GET_A           = 5     # Get motor 1 direction and PWM rate
COMMAND_SET_B_FWD       = 6     # Set motor 2 PWM rate in a forwards direction
COMMAND_SET_B_REV       = 7     # Set motor 2 PWM rate in a reverse direction
COMMAND_GET_B           = 8     # Get motor 2 direction and PWM rate
COMMAND_SET_C_FWD       = 9     # Set motor 3 PWM rate in a forwards direction
COMMAND_SET_C_REV       = 10    # Set motor 3 PWM rate in a reverse direction
COMMAND_GET_C           = 11    # Get motor 3 direction and PWM rate
COMMAND_SET_D_FWD       = 12    # Set motor 4 PWM rate in a forwards direction
COMMAND_SET_D_REV       = 13    # Set motor 4 PWM rate in a reverse direction
COMMAND_GET_D           = 14    # Get motor 4 direction and PWM rate
COMMAND_ALL_OFF         = 15    # Switch everything off
COMMAND_SET_ALL_FWD     = 16    # Set all motors PWM rate in a forwards direction
COMMAND_SET_ALL_REV     = 17    # Set all motors PWM rate in a reverse direction
COMMAND_SET_FAILSAFE    = 18    # Set the failsafe flag, turns the motors off if communication is interrupted
COMMAND_GET_FAILSAFE    = 19    # Get the failsafe flag
COMMAND_RESET_EPO       = 20    # Resets the EPO flag, use after EPO has been tripped and switch is now clear
COMMAND_GET_EPO         = 21    # Get the EPO latched flag
COMMAND_SET_EPO_IGNORE  = 22    # Set the EPO ignored flag, allows the system to run without an EPO
COMMAND_GET_EPO_IGNORE  = 23    # Get the EPO ignored flag
COMMAND_GET_NEW_IR      = 24    # Get the new IR message received flag
COMMAND_GET_LAST_IR     = 25    # Get the last IR message received (long message, resets new IR flag)
COMMAND_SET_LED_IR      = 26    # Set the LED for indicating IR messages
COMMAND_GET_LED_IR      = 27    # Get if the LED is being used to indicate IR messages
COMMAND_GET_ANALOG_1    = 28    # Get the analog reading from port #1, pin 2
COMMAND_GET_ANALOG_2    = 29    # Get the analog reading from port #2, pin 4
COMMAND_GET_ID          = 0x99  # Get the board identifier
COMMAND_SET_I2C_ADD     = 0xAA  # Set a new I2C address

COMMAND_VALUE_FWD       = 1     # I2C value representing forward
COMMAND_VALUE_REV       = 2     # I2C value representing reverse

COMMAND_VALUE_ON        = 1     # I2C value representing on
COMMAND_VALUE_OFF       = 0     # I2C value representing off

COMMAND_ANALOG_MAX      = 0x3FF # Maximum value for analog readings

IR_MAX_BYTES            = I2C_LONG_LEN - 2


def scan_for_zero_borg(bus_number=1):
    """
scan_for_zero_borg([bus_number])

Scans the I²C bus for a ZeroBorg boards and returns a list of all usable addresses
The bus_number if supplied is which I²C bus to scan, 0 for Rev 1 boards, 1 for Rev 2 boards, if not supplied
the default is 1
    """
    found = []
    print('Scanning I²C bus #%d' % bus_number)
    bus = ZeroBorg()
    for address in range(0x03, 0x78, 1):
        try:
            bus.init_bus_only(bus_number, address)
            i2c_recv = bus.raw_read(COMMAND_GET_ID, I2C_NORM_LEN)
            if len(i2c_recv) == I2C_NORM_LEN:
                if i2c_recv[1] == I2C_ID_ZEROBORG:
                    print('Found ZeroBorg at %02X' % address)
                    found.append(address)
                else:
                    pass
            else:
                pass
        except KeyboardInterrupt:
            raise
        except:
            pass
    if len(found) == 0:
        print('No ZeroBorg boards found, is bus #%d correct (should be 0 for Rev 1, 1 for Rev 2)' % bus_number)
    elif len(found) == 1:
        print('1 ZeroBorg board found')
    else:
        print('%d ZeroBorg boards found' % (len(found)))
    return found


def set_new_address(new_address, old_address=-1, bus_number=1):
    """
SetNewAddress(new_address, [old_address], [bus_number])

Scans the I²C bus for the first ZeroBorg and sets it to a new I2C address
If old_address is supplied it will change the address of the board at that address rather than scanning the bus
The bus_number if supplied is which I²C bus to scan, 0 for Rev 1 boards, 1 for Rev 2 boards, if not supplied
the default is 1
Warning, this new I²C address will still be used after resetting the power on the device
    """
    if new_address < 0x03:
        print('Error, I²C addresses below 3 (0x03) are reserved, use an address between 3 (0x03) and 119 (0x77)')
        return
    elif new_address > 0x77:
        print('Error, I²C addresses above 119 (0x77) are reserved, use an address between 3 (0x03) and 119 (0x77)')
        return
    if old_address < 0x0:
        found = scan_for_zero_borg(bus_number)
        if len(found) < 1:
            print('No ZeroBorg boards found, cannot set a new I²C address!')
            return
        else:
            old_address = found[0]
    print('Changing I²C address from %02X to %02X (bus #%d)' % (old_address, new_address, bus_number))
    bus = ZeroBorg()
    bus.init_bus_only(bus_number, old_address)
    try:
        i2c_recv = bus.raw_read(COMMAND_GET_ID, I2C_NORM_LEN)
        if len(i2c_recv) == I2C_NORM_LEN:
            if i2c_recv[1] == I2C_ID_ZEROBORG:
                found_chip = True
                print('Found ZeroBorg at %02X' % old_address)
            else:
                found_chip = False
                print('Found a device at %02X, but it is not a ZeroBorg (ID %02X instead of %02X)' % (
                    old_address, i2c_recv[1], I2C_ID_ZEROBORG))
        else:
            found_chip = False
            print('Missing ZeroBorg at %02X' % old_address)
    except KeyboardInterrupt:
        raise
    except:
        found_chip = False
        print('Missing ZeroBorg at %02X' % old_address)
    if found_chip:
        bus.raw_write(COMMAND_SET_I2C_ADD, [new_address])
        time.sleep(0.1)
        print('Address changed to %02X, attempting to talk with the new address' % new_address)
        try:
            bus.init_bus_only(bus_number, new_address)
            i2c_recv = bus.raw_read(COMMAND_GET_ID, I2C_NORM_LEN)
            if len(i2c_recv) == I2C_NORM_LEN:
                if i2c_recv[1] == I2C_ID_ZEROBORG:
                    found_chip = True
                    print('Found ZeroBorg at %02X' % new_address)
                else:
                    found_chip = False
                    print('Found a device at %02X, but it is not a ZeroBorg (ID %02X instead of %02X)' % (
                        new_address, i2c_recv[1], I2C_ID_ZEROBORG))
            else:
                found_chip = False
                print('Missing ZeroBorg at %02X' % new_address)
        except KeyboardInterrupt:
            raise
        except:
            found_chip = False
            print('Missing ZeroBorg at %02X' % new_address)
    if found_chip:
        print('New I²C address of %02X set successfully' % new_address)
    else:
        print('Failed to set new I²C address...')


# Class used to control ZeroBorg
class ZeroBorg:
    """
This module is designed to communicate with the ZeroBorg

bus_number               I²C bus on which the ZeroBorg is attached (Rev 1 is bus 0, Rev 2 is bus 1)
bus                     the smbus object used to talk to the I²C bus
i2c_address              The I²C address of the ZeroBorg chip to control
foundChip               True if the ZeroBorg chip can be seen, False otherwise
printFunction           Function reference to call when printing text, if None "print" is used
    """

    # Shared values used by this class
    bus_number = 1  # Check here for Rev 1 vs Rev 2 and select the correct bus
    i2c_address = I2C_ID_ZEROBORG  # I²C address, override for a different address
    foundChip = False
    printFunction = None
    i2cWrite = None
    i2c_read = None

    def raw_write(self, command, data):
        """
raw_write(command, data)

Sends a raw command on the I2C bus to the ZeroBorg
Command codes can be found at the top of ZeroBorg.py, data is a list of 0 or more byte values

Under most circumstances you should use the appropriate function instead of raw_write
        """
        raw_output = chr(command)
        for single_byte in data:
            raw_output += chr(single_byte)
        self.i2cWrite.write(raw_output)

    def raw_read(self, command, length, retry_count = 3):
        """
raw_read(command, length, [retry_count])

Reads data back from the ZeroBorg after sending a GET command
Command codes can be found at the top of ZeroBorg.py, length is the number of bytes to read back

The function checks that the first byte read back matches the requested command
If it does not it will retry the request until retry_count is exhausted (default is 3 times)

Under most circumstances you should use the appropriate function instead of raw_read
        """
        while retry_count > 0:
            self.raw_write(command, [])
            raw_reply = self.i2c_read.read(length)
            reply = []
            for single_byte in raw_reply:
                reply.append(ord(single_byte))
            if command == reply[0]:
                break
            else:
                retry_count -= 1
        if retry_count > 0:
            return reply
        else:
            raise IOError('I2C read for command %d failed' % command)

    def init_bus_only(self, bus_number, address):
        """
init_bus_only(bus_number, address)

Prepare the I2C driver for talking to a ZeroBorg on the specified bus and I2C address
This call does not check the board is present or working, under most circumstances use Init() instead
        """
        self.bus_number = bus_number
        self.i2c_address = address
        self.i2c_read = io.open("/dev/i2c-" + str(self.bus_number), "rb", buffering = 0)
        fcntl.ioctl(self.i2c_read, I2C_SLAVE, self.i2c_address)
        self.i2cWrite = io.open("/dev/i2c-" + str(self.bus_number), "wb", buffering = 0)
        fcntl.ioctl(self.i2cWrite, I2C_SLAVE, self.i2c_address)

    def Print(self, message):
        """
Print(message)

Wrapper used by the ZeroBorg instance to print(messages, will call printFunction if set, print(otherwise
        """
        if self.printFunction is None:
            print(message)
        else:
            self.printFunction(message)

    def NoPrint(self, message):
        """
NoPrint(message)

Does nothing, intended for disabling diagnostic printout by using:
ZB = ZeroBorg.ZeroBorg()
ZB.printFunction = ZB.NoPrint
        """
        pass

    def init(self, try_other_bus=True):
        """
init([try_other_bus])

Prepare the I2C driver for talking to the ZeroBorg
If try_other_bus is True or omitted, this function will attempt to use the other bus if the ZeroBorg devices
can not be found on the current bus_number
        """
        self.Print('Loading ZeroBorg on bus %d, address %02X' % (self.bus_number, self.i2c_address))

        # Check for ZeroBorg
        try:
            # Open the bus
            self.i2c_read = io.open("/dev/i2c-" + str(self.bus_number), "rb", buffering=0)
            fcntl.ioctl(self.i2c_read, I2C_SLAVE, self.i2c_address)
            self.i2cWrite = io.open("/dev/i2c-" + str(self.bus_number), "wb", buffering=0)
            fcntl.ioctl(self.i2cWrite, I2C_SLAVE, self.i2c_address)
            i2c_recv = self.raw_read(COMMAND_GET_ID, I2C_NORM_LEN)
            if len(i2c_recv) == I2C_NORM_LEN:
                if i2c_recv[1] == I2C_ID_ZEROBORG:
                    self.foundChip = True
                    self.Print('Found ZeroBorg at %02X' % self.i2c_address)
                else:
                    self.foundChip = False
                    self.Print('Found a device at %02X, but it is not a ZeroBorg (ID %02X instead of %02X)' % (
                        self.i2c_address, i2c_recv[1], I2C_ID_ZEROBORG))
            else:
                self.foundChip = False
                self.Print('Missing ZeroBorg at %02X' % self.i2c_address)
        except KeyboardInterrupt:
            raise
        except:
            self.foundChip = False
            self.Print('Missing ZeroBorg at %02X' % self.i2c_address)

        # See if we are missing chips
        if not self.foundChip:
            self.Print('ZeroBorg was not found')
            if try_other_bus:
                if self.bus_number == 1:
                    self.bus_number = 0
                else:
                    self.bus_number = 1
                self.Print('Trying bus %d instead' % self.bus_number)
                self.init(False)
            else:
                self.Print(
                    'Are you sure your ZeroBorg is properly attached, the correct address is used, and '
                    'the I2C drivers are running?')
                self.bus = None
        else:
            self.Print('ZeroBorg loaded on bus %d' % self.bus_number)

    def set_motor1(self, power):
        """
set_motor1(power)

Sets the drive level for motor 1, from +1 to -1.
e.g.
set_motor1(0)     -> motor 1 is stopped
set_motor1(0.75)  -> motor 1 moving forward at 75% power
set_motor1(-0.5)  -> motor 1 moving reverse at 50% power
set_motor1(1)     -> motor 1 moving forward at 100% power
        """
        if power < 0:
            # Reverse
            command = COMMAND_SET_A_REV
            pwm = -int(PWM_MAX * power)
            if pwm > PWM_MAX:
                pwm = PWM_MAX
        else:
            # Forward / stopped
            command = COMMAND_SET_A_FWD
            pwm = int(PWM_MAX * power)
            if pwm > PWM_MAX:
                pwm = PWM_MAX

        try:
            self.bus.write_byte_data(self.i2c_address, command, pwm)
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed sending motor 1 drive level!')

    def get_motor1(self):
        """
power = get_motor1()

Gets the drive level for motor 1, from +1 to -1.
e.g.
0     -> motor 1 is stopped
0.75  -> motor 1 moving forward at 75% power
-0.5  -> motor 1 moving reverse at 50% power
1     -> motor 1 moving forward at 100% power
        """
        try:
            i2c_recv = self.bus.read_i2c_block_data(self.i2c_address, COMMAND_GET_A, I2C_NORM_LEN)
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed reading motor 1 drive level!')
            return

        power = float(i2c_recv[2]) / float(PWM_MAX)

        if i2c_recv[1] == COMMAND_VALUE_FWD:
            return power
        elif i2c_recv[1] == COMMAND_VALUE_REV:
            return -power
        else:
            return

    def set_motor2(self, power):
        """
set_motor1(power)

Sets the drive level for motor 2, from +1 to -1.
e.g.
set_motor2(0)     -> motor 2 is stopped
set_motor2(0.75)  -> motor 2 moving forward at 75% power
set_motor2(-0.5)  -> motor 2 moving reverse at 50% power
set_motor2(1)     -> motor 2 moving forward at 100% power
        """
        if power < 0:
            # Reverse
            command = COMMAND_SET_B_REV
            pwm = -int(PWM_MAX * power)
            if pwm > PWM_MAX:
                pwm = PWM_MAX
        else:
            # Forward / stopped
            command = COMMAND_SET_B_FWD
            pwm = int(PWM_MAX * power)
            if pwm > PWM_MAX:
                pwm = PWM_MAX

        try:
            self.bus.write_byte_data(self.i2c_address, command, pwm)
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed sending motor 2 drive level!')

    def get_motor2(self):
        """
power = get_motor2()

Gets the drive level for motor 2, from +1 to -1.
e.g.
0     -> motor 2 is stopped
0.75  -> motor 2 moving forward at 75% power
-0.5  -> motor 2 moving reverse at 50% power
1     -> motor 2 moving forward at 100% power
        """
        try:
            i2c_recv = self.bus.read_i2c_block_data(self.i2c_address, COMMAND_GET_B, I2C_NORM_LEN)
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed reading motor 2 drive level!')
            return

        power = float(i2c_recv[2]) / float(PWM_MAX)

        if i2c_recv[1] == COMMAND_VALUE_FWD:
            return power
        elif i2c_recv[1] == COMMAND_VALUE_REV:
            return -power
        else:
            return

    def set_motor3(self, power):
        """
set_motor3(power)

Sets the drive level for motor 3, from +1 to -1.
e.g.
set_motor3(0)     -> motor 3 is stopped
set_motor3(0.75)  -> motor 3 moving forward at 75% power
set_motor3(-0.5)  -> motor 3 moving reverse at 50% power
set_motor3(1)     -> motor 3 moving forward at 100% power
        """
        if power < 0:
            # Reverse
            command = COMMAND_SET_C_REV
            pwm = -int(PWM_MAX * power)
            if pwm > PWM_MAX:
                pwm = PWM_MAX
        else:
            # Forward / stopped
            command = COMMAND_SET_C_FWD
            pwm = int(PWM_MAX * power)
            if pwm > PWM_MAX:
                pwm = PWM_MAX

        try:
            self.bus.write_byte_data(self.i2c_address, command, pwm)
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed sending motor 3 drive level!')

    def i2c_recv(self):
        """
power = get_motor3()

Gets the drive level for motor 3, from +1 to -1.
e.g.
0     -> motor 3 is stopped
0.75  -> motor 3 moving forward at 75% power
-0.5  -> motor 3 moving reverse at 50% power
1     -> motor 3 moving forward at 100% power
        """
        try:
            i2c_recv = self.bus.read_i2c_block_data(self.i2c_address, COMMAND_GET_C, I2C_NORM_LEN)
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed reading motor 3 drive level!')
            return

        power = float(i2c_recv[2]) / float(PWM_MAX)

        if i2c_recv[1] == COMMAND_VALUE_FWD:
            return power
        elif i2c_recv[1] == COMMAND_VALUE_REV:
            return -power
        else:
            return

    def set_motor4(self, power):
        """
set_motor4(power)

Sets the drive level for motor 4, from +1 to -1.
e.g.
set_motor4(0)     -> motor 4 is stopped
set_motor4(0.75)  -> motor 4 moving forward at 75% power
set_motor4(-0.5)  -> motor 4 moving reverse at 50% power
set_motor4(1)     -> motor 4 moving forward at 100% power
        """
        if power < 0:
            # Reverse
            command = COMMAND_SET_D_REV
            pwm = -int(PWM_MAX * power)
            if pwm > PWM_MAX:
                pwm = PWM_MAX
        else:
            # Forward / stopped
            command = COMMAND_SET_D_FWD
            pwm = int(PWM_MAX * power)
            if pwm > PWM_MAX:
                pwm = PWM_MAX

        try:
            self.bus.write_byte_data(self.i2c_address, command, pwm)
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed sending motor 4 drive level!')

    def get_motor4(self):
        """
power = get_motor4()

Gets the drive level for motor 4, from +1 to -1.
e.g.
0     -> motor 4 is stopped
0.75  -> motor 4 moving forward at 75% power
-0.5  -> motor 4 moving reverse at 50% power
1     -> motor 4 moving forward at 100% power
        """
        try:
            i2c_recv = self.bus.read_i2c_block_data(self.i2c_address, COMMAND_GET_D, I2C_NORM_LEN)
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed reading motor 4 drive level!')
            return

        power = float(i2c_recv[2]) / float(PWM_MAX)

        if i2c_recv[1] == COMMAND_VALUE_FWD:
            return power
        elif i2c_recv[1] == COMMAND_VALUE_REV:
            return -power
        else:
            return

    def set_motors(self, power):
        """
set_motors(power)

Sets the drive level for all motors, from +1 to -1.
e.g.
set_motors(0)     -> all motors are stopped
set_motors(0.75)  -> all motors are moving forward at 75% power
set_motors(-0.5)  -> all motors are moving reverse at 50% power
set_motors(1)     -> all motors are moving forward at 100% power
        """
        if power < 0:
            # Reverse
            command = COMMAND_SET_ALL_REV
            pwm = -int(PWM_MAX * power)
            if pwm > PWM_MAX:
                pwm = PWM_MAX
        else:
            # Forward / stopped
            command = COMMAND_SET_ALL_FWD
            pwm = int(PWM_MAX * power)
            if pwm > PWM_MAX:
                pwm = PWM_MAX

        try:
            self.bus.write_byte_data(self.i2c_address, command, pwm)
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed sending all motors drive level!')

    def motors_off(self):
        """
motors_off()

Sets all motors to stopped, useful when ending a program
        """
        try:
            self.bus.write_byte_data(self.i2c_address, COMMAND_ALL_OFF, 0)
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed sending motors off command!')

    def set_led(self, state):
        """
set_led(state)

Sets the current state of the LED, False for off, True for on
        """
        if state:
            level = COMMAND_VALUE_ON
        else:
            level = COMMAND_VALUE_OFF

        try:
            self.bus.write_byte_data(self.i2c_address, COMMAND_SET_LED, level)
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed sending LED state!')

    def get_led(self):
        """
state = get_led()

Reads the current state of the LED, False for off, True for on
        """
        try:
            i2c_recv = self.bus.read_i2c_block_data(self.i2c_address, COMMAND_GET_LED, I2C_NORM_LEN)
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed reading LED state!')
            return

        if i2c_recv[1] == COMMAND_VALUE_OFF:
            return False
        else:
            return True

    def reset_epo(self):
        """
reset_epo()

Resets the EPO latch state, use to allow movement again after the EPO has been tripped
        """
        try:
            self.bus.write_byte_data(self.i2c_address, COMMAND_RESET_EPO, 0)
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed resetting EPO!')

    def get_epo(self):
        """
state = get_epo()

Reads the system EPO latch state.
If False the EPO has not been tripped, and movement is allowed.
If True the EPO has been tripped, movement is disabled if the EPO is not ignored (see set_epo_ignore)
    Movement can be re-enabled by calling reset_epo.
        """
        try:
            i2c_recv = self.bus.read_i2c_block_data(self.i2c_address, COMMAND_GET_EPO, I2C_NORM_LEN)
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed reading EPO ignore state!')
            return

        if i2c_recv[1] == COMMAND_VALUE_OFF:
            return False
        else:
            return True

    def set_epo_ignore(self, state):
        """
set_epo_ignore(state)

Sets the system to ignore or use the EPO latch, set to False if you have an EPO switch, True if you do not
        """
        if state:
            level = COMMAND_VALUE_ON
        else:
            level = COMMAND_VALUE_OFF

        try:
            self.bus.write_byte_data(self.i2c_address, COMMAND_SET_EPO_IGNORE, level)
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed sending EPO ignore state!')

    def get_epo_ignore(self):
        """
state = get_epo_ignore()

Reads the system EPO ignore state, False for using the EPO latch, True for ignoring the EPO latch
        """
        try:
            i2c_recv = self.bus.read_i2c_block_data(self.i2c_address, COMMAND_GET_EPO_IGNORE, I2C_NORM_LEN)
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed reading EPO ignore state!')
            return

        if i2c_recv[1] == COMMAND_VALUE_OFF:
            return False
        else:
            return True

    def has_new_ir_message(self):
        """
state = has_new_ir_message()

Reads the new IR message received flag.
If False there has been no messages to the IR sensor since the last read.
If True there has been a new IR message which can be read using get_ir_message().
        """
        try:
            i2c_recv = self.bus.read_i2c_block_data(self.i2c_address, COMMAND_GET_NEW_IR, I2C_NORM_LEN)
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed reading new IR message received flag!')
            return

        if i2c_recv[1] == COMMAND_VALUE_OFF:
            return False
        else:
            return True

    def get_ir_message(self):
        """
message = get_ir_message()

Reads the last IR message which has been received and clears the new IR message received flag.
Returns the bytes from the remote control as a hexadecimal string, e.g. 'F75AD5AA8000'
Use has_new_ir_message() to see if there has been a new IR message since the last call.
        """
        try:
            i2c_recv = self.bus.read_i2c_block_data(self.i2c_address, COMMAND_GET_LAST_IR, I2C_LONG_LEN)
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed reading IR message!')
            return

        message = ''
        for i in range(IR_MAX_BYTES):
            message += '%02X' % (i2c_recv[1 + i])
        return message.rstrip('0')

    def set_led_ir(self, state):
        """
set_led_ir(state)

Sets if IR messages control the state of the LED, False for no effect, True for incoming messages blink the LED
        """
        if state:
            level = COMMAND_VALUE_ON
        else:
            level = COMMAND_VALUE_OFF

        try:
            self.bus.write_byte_data(self.i2c_address, COMMAND_SET_LED_IR, level)
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed sending LED state!')

    def get_led_ir(self):
        """
state = get_led_ir()

Reads if IR messages control the state of the LED, False for no effect, True for incoming messages blink the LED
        """
        try:
            i2c_recv = self.bus.read_i2c_block_data(self.i2c_address, COMMAND_GET_LED_IR, I2C_NORM_LEN)
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed reading LED state!')
            return

        if i2c_recv[1] == COMMAND_VALUE_OFF:
            return False
        else:
            return True

    def get_analog1(self):
        """
voltage = get_analog1()

Reads the current analog level from port #1 (pin 2).
Returns the value as a voltage based on the 3.3 V reference pin (pin 1).
        """
        try:
            i2c_recv = self.bus.read_i2c_block_data(self.i2c_address, COMMAND_GET_ANALOG_1, I2C_NORM_LEN)
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed reading analog level #1!')
            return

        raw = (i2c_recv[1] << 8) + i2c_recv[2]
        level = float(raw) / float(COMMAND_ANALOG_MAX)
        return level * 3.3

    def get_analog2(self):
        """
voltage = get_analog2()

Reads the current analog level from port #2 (pin 4).
Returns the value as a voltage based on the 3.3 V reference pin (pin 1).
        """
        try:
            i2c_recv = self.bus.read_i2c_block_data(self.i2c_address, COMMAND_GET_ANALOG_2, I2C_NORM_LEN)
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed reading analog level #2!')
            return

        raw = (i2c_recv[1] << 8) + i2c_recv[2]
        level = float(raw) / float(COMMAND_ANALOG_MAX)
        return level * 3.3

    def set_comms_failsafe(self, state):
        """
set_comms_failsafe(state)

Sets the system to enable or disable the communications failsafe
The failsafe will turn the motors off unless it is commanded at least once every 1/4 of a second
Set to True to enable this failsafe, set to False to disable this failsafe
The failsafe is disabled at power on
        """
        if state:
            level = COMMAND_VALUE_ON
        else:
            level = COMMAND_VALUE_OFF

        try:
            self.bus.write_byte_data(self.i2c_address, COMMAND_SET_FAILSAFE, level)
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed sending communications failsafe state!')

    def get_comms_failsafe(self):
        """
state = get_comms_failsafe()

Read the current system state of the communications failsafe, True for enabled, False for disabled
The failsafe will turn the motors off unless it is commanded at least once every 1/4 of a second
        """
        try:
            i2c_recv = self.bus.read_i2c_block_data(self.i2c_address, COMMAND_GET_FAILSAFE, I2C_NORM_LEN)
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed reading communications failsafe state!')
            return

        if i2c_recv[1] == COMMAND_VALUE_OFF:
            return False
        else:
            return True

    def help(self):
        """
help()

Displays the names and descriptions of the various functions and settings provided
        """
        func_list = [ZeroBorg.__dict__.get(a) for a in dir(ZeroBorg) if
                    isinstance(ZeroBorg.__dict__.get(a), types.FunctionType)]
        func_list_sorted = sorted(func_list, key=lambda x: x.func_code.co_firstlineno)

        print(self.__doc__)
        print()
        for func in func_list_sorted:
            print('=== %s === %s' % (func.func_name, func.func_doc))
