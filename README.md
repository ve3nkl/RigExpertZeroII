## Micropython RigExpert Zero II module

This Micropython module allows you to communicate with RigExpert ZERO II board from RPi Pico or RPi Pico W using the I2C interface. 

### Usage example

Note, that this example is using default values for RPi Pico I2C pins. Depending on what pins you intend to use, you might need to specify I2C\_ID, SCL\_PIN, SDA\_PIN and RESET\_PIN when creating the RigExpertZeroII object.
  
    from RigExpertZeroII import RigExpertZeroII
    z = RigExpertZeroII()
    if z.start_zeroii():
        print(z.get_major_version())
        print(z.get_minor_version())
        print(z.get_hw_revision())
        print(z.get_serial_number())
        print(z.get_system_z0())
        print(z.get_status())
        for f in range(144000000, 148000000, 100000):
            if z.measure(f):
                print("f=" + str(f) + ", " + \
                str(z.get_obtained_swr()) + ", " + \
                str(z.get_obtained_mismatch_loss()) + \
                ", " + str(z.get_obtained_r()) + ", " + \
                str(z.get_obtained_x()))
            else:
                print("Initializing ZEROII board failed...") 

### A Word on Physical Connections...
Since RPi Pico is using 3.3V logic levels while ZERO II board is using 5V logic levels, it is recommended to use a logic level converter integrated circuit for SDA, SCL and RESET connections. Also, pay close attention to the position of the board's power mode switch. It allows you to select between USB and board '+'/'-' pin power. My personal experience of experimenting with the board made me believe that this switch should be shifted into the position for the external power and the power (5V) should be applied to the '+'/'-' pins. I did not have much luck attempting to use the board powered through USB connector. Your experience may be different...
