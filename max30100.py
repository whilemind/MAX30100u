import smbus
import math
import time

INT_STATUS   = 0x00  # Which interrupts are tripped
INT_ENABLE   = 0x01  # Which interrupts are active
FIFO_WR_PTR  = 0x02  # Where data is being written
OVRFLOW_CTR  = 0x03  # Number of lost samples
FIFO_RD_PTR  = 0x04  # Where to read from
FIFO_DATA    = 0x05  # Ouput data buffer
MODE_CONFIG  = 0x06  # Control register
SPO2_CONFIG  = 0x07  # Oximetry settings
LED_CONFIG   = 0x09  # Pulse width and power of LEDs
TEMP_INTG    = 0x16  # Temperature value, whole number
TEMP_FRAC    = 0x17  # Temperature value, fraction
REV_ID       = 0xFE  # Part revision
PART_ID      = 0xFF  # Part ID, normally 0x11

I2C_ADDRESS  = 0x57  # I2C address of the MAX30100 device


PULSE_WIDTH = {
    200: 0,
    400: 1,
    800: 2,
   1600: 3,
}

SAMPLE_RATE = {
    50: 0,
   100: 1,
   167: 2,
   200: 3,
   400: 4,
   600: 5,
   800: 6,
  1000: 7,
}

LED_CURRENT = {
       0: 0,
     4.4: 1,
     7.6: 2,
    11.0: 3,
    14.2: 4,
    17.4: 5,
    20.8: 6,
    24.0: 7,
    27.1: 8,
    30.6: 9,
    33.8: 10,
    37.0: 11,
    40.2: 12,
    43.6: 13,
    46.8: 14,
    50.0: 15
}

def _get_valid(d, value):
    try:
        return d[value]
    except KeyError:
        raise KeyError("Value %s not valid, use one of: %s" % (value, ', '.join([str(s) for s in d.keys()])))

def _twos_complement(val, bits):
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)
    return val

INTERRUPT_SPO2 = 0
INTERRUPT_HR = 1
INTERRUPT_TEMP = 2
INTERRUPT_FIFO = 3

MODE_HR = 0x02
MODE_SPO2 = 0x03


class pulseoxymeter_t(object):
    def __init__(self,
        pulseDetected = False,
        heartBPM = 0.0,
        irCardiogram = 0.0,
        irDcValue = 0.0,
        redDcValue = 0.0,
        currentSaO2Value = 0.0,
        lastBeatThreshold = 0.0,
        dcFilteredIR = 0.0,
        dcFilteredRed = 0.0
        ):
        self.pulseDetected = pulseDetected
        self.heartBPM = heartBPM,
        self.irCardiogram = irCardiogram,
        self.irDcValue = irDcValue,
        self.redDcValue = redDcValue,
        self.SaO2 = currentSaO2Value,
        self.lastBeatThreshold = lastBeatThreshold,
        self.dcFilteredIR = dcFilteredIR,
        self.dcFilteredRed = 0.0


class dcFilter_t(object): 
    def __init__(self, w=0.0, result=0.0):
        self.w = w
        self.result = result


class meanDiffFilter_t(object):
    def __init__(self, index = 0, sum = 0, count = 0):
        self.values = range(15)
        self.index = index
        self.sum = sum
        self.count = count

class butterworthFilter_t(object):
    def __init__(self, result=0.0):
        self.v = range(2)
        self.result = result


class MAX30100(object):

    def __init__(self,
                 i2c=None,
                 mode=MODE_HR,
                 sample_rate=100,
                 led_current_red=27.1,
                 led_current_ir=27.1,
                 pulse_width=1600,
                 max_buffer_len=10000,
                 debug=False
                 ):

        # Default to the standard I2C bus on Pi.
        self.i2c = i2c if i2c else smbus.SMBus(1)

        self.set_mode(MODE_HR)  # Trigger an initial temperature read.
        self.set_led_current(led_current_red, led_current_ir)
        self.set_spo_config(sample_rate, pulse_width)

        # Reflectance data (latest update)
        self.led_current_ir = led_current_ir
        self.led_current_red = led_current_red
        self.i2cValue = i2c
        self.sample_rate = sample_rate
        self.pulse_width = pulse_width
        self.max_buffer_len = max_buffer_len

        self.buffer_red = []
        self.buffer_ir = []

        self._interrupt = None


        # Update classes for BPM OS2 calcualtion.
        # This are enum type
        self.PULSE_IDLE = 0
        self.PULSE_TRACE_UP = 1
        self.PULSE_TRACE_DOWN = 2

        self.MAGIC_ACCEPTABLE_INTENSITY_DIFF = 65000
        self.RED_LED_CURRENT_ADJUSTMENT_MS = 500
        self.RESET_SPO2_EVERY_N_PULSES = 4
        self.ALPHA = 0.95
        self.MEAN_FILTER_SIZE = 15  

        self.PULSE_MIN_THRESHOLD = 100 #300 is good for finger, but for wrist you need like 20, and there is shitloads of noise
        self.PULSE_MAX_THRESHOLD = 2000
        self.PULSE_GO_DOWN_THRESHOLD = 1
        self.PULSE_BPM_SAMPLE_SIZE = 10 

        self.debug = debug
        self.dcFilterIR = dcFilter_t()
        self.dcFilterRed = dcFilter_t()
        self.meanDiffIR = meanDiffFilter_t()
        self.meanDiffIR = meanDiffFilter_t()
        self.lpbFilterIR = butterworthFilter_t()
        self.irACValueSqSum = 0
        self.redACValueSqSum = 0
        self.samplesRecorded = 0
        self.pulsesDetected = 0
        self.currentSaO2Value = 0
        self.currentPulseDetectorState = self.PULSE_IDLE
        self.lastBeatThreshold = 0
        self.currentBPM = 0
        
        self.valuesBPM = range(self.PULSE_BPM_SAMPLE_SIZE)
        self.valuesBPMSum = 0
        self.valuesBPMCount = 0
        self.bpmIndex = 0




    def reinit(self):
        self.i2c = self.i2cValue if self.i2cValue else smbus.SMBus(1)

        self.set_mode(MODE_HR)  # Trigger an initial temperature read.
        self.set_led_current(self.led_current_red, self.led_current_ir)
        self.set_spo_config(self.sample_rate, self.pulse_width)

        # Reflectance data (latest update)
        self.buffer_red = []
        self.buffer_ir = []

        self.max_buffer_len = 10000
        self._interrupt = None


    def update(self):
        result = pulseoxymeter_t(currentSaO2Value=self.currentSaO2Value)
        self.read_sensor()
        
        self.dcFilterIR = self.dcRemoval(self.ir, self.dcFilterIR.w, self.ALPHA)
        self.dcFilterRed = self.dcRemoval(self.red, self.dcFilterRed.w, self.ALPHA)       

        
        (meanDiffResIR, self.meanDiffIR) = self.meanDiff(self.dcFilterIR.result, self.meanDiffIR)
    
        if(self.debug):
            print("Ir {} and red {}".format(self.ir, self.red))
            print("dcFilterIR w {} result {}".format(self.dcFilterIR.w, self.dcFilterIR.result))
            print("dcFilterRed w {} result {}".format(self.dcFilterRed.w, self.dcFilterRed.result))
            print("meanDiffResIR {} IR filter {}".format(meanDiffResIR, self.meanDiffIR.values))

        self.lpbFilterIR = self.lowPassButterworthFilter( meanDiffResIR, self.lpbFilterIR )
        
        self.irACValueSqSum = self.irACValueSqSum + (self.dcFilterIR.result * self.dcFilterIR.result)
        self.redACValueSqSum = self.redACValueSqSum  + (self.dcFilterRed.result * self.dcFilterRed.result)
        self.samplesRecorded = self.samplesRecorded + 1

        if( self.detectPulse( self.lpbFilterIR.result ) and self.samplesRecorded > 0 ):
            result.pulseDetected = True
            self.pulsesDetected = self.pulsesDetected + 1

            ratioRMS = math.log( math.sqrt(self.redACValueSqSum / self.samplesRecorded), 10 ) / math.log( math.sqrt(self.irACValueSqSum / self.samplesRecorded), 10 )

            if(self.debug):
                print("RMS Ratio: {}".format(str(ratioRMS)))

            # //This is my adjusted standard model, so it shows 0.89 as 94% saturation. It is probably far from correct, requires proper empircal calibration
            self.currentSaO2Value = 110.0 - 18.0 * ratioRMS
            result.SaO2 = self.currentSaO2Value
            
            if( self.pulsesDetected % self.RESET_SPO2_EVERY_N_PULSES == 0):
                self.irACValueSqSum = 0
                self.redACValueSqSum = 0
                self.samplesRecorded = 0
        else:
            if(self.debug):
                print("Not entring detectPulse!!!!")


        return result


    def dcRemoval(self, x, prev_w, alpha):
        filtered = dcFilter_t()
        filtered.w = x + alpha * prev_w
        filtered.result = filtered.w - prev_w

        return filtered


    def meanDiff(self, M, filterValues):
        avg = 0

        filterValues.sum = filterValues.sum - filterValues.values[filterValues.index]
        filterValues.values[filterValues.index] = M
        filterValues.sum = filterValues.sum + filterValues.values[filterValues.index]

        filterValues.index = filterValues.index + 1
        filterValues.index = filterValues.index % self.MEAN_FILTER_SIZE

        if(filterValues.count < self.MEAN_FILTER_SIZE):
            filterValues.count = filterValues.count + 1

        avg = filterValues.sum / filterValues.count
        
        return (avg - M, filterValues)

    def lowPassButterworthFilter(self, x, filterResult):
        filterResult.v[0] = filterResult.v[1]

        # Fs = 100Hz and Fc = 10Hz
        filterResult.v[1] = (2.452372752527856026e-1 * x) + (0.50952544949442879485 * filterResult.v[0])

        # Fs = 100Hz and Fc = 4Hz
        # filterResult.v[1] = (1.367287359973195227e-1 * x) + (0.72654252800536101020 * filterResult.v[0]) #Very precise butterworth filter 

        filterResult.result = filterResult.v[0] + filterResult.v[1]

        return filterResult


    def detectPulse(self, sensor_value):
        if(self.debug):
            print("Inside detectPulse with {} current pulse state {}".format(sensor_value, self.currentPulseDetectorState))

        # why this variables are static
        prev_sensor_value = 0
        values_went_down = 0
        currentBeat = 0
        lastBeat = 0

        if(sensor_value > self.PULSE_MAX_THRESHOLD):
            self.currentPulseDetectorState = self.PULSE_IDLE
            prev_sensor_value = 0
            lastBeat = 0
            currentBeat = 0
            values_went_down = 0
            lastBeatThreshold = 0
            return False


        if(self.currentPulseDetectorState == self.PULSE_IDLE):
            if(sensor_value >= self.PULSE_MIN_THRESHOLD): 
                self.currentPulseDetectorState = self.PULSE_TRACE_UP
                values_went_down = 0

        elif(self.currentPulseDetectorState == self.PULSE_TRACE_UP):
            if(sensor_value > prev_sensor_value):
                currentBeat = lambda: int(round(time.time() * 1000))
                self.lastBeatThreshold = sensor_value
            else:
                if(self.debug == True): 
                    print("Peak reached: {} and {}".format(sensor_value, prev_sensor_value))

                beatDuration = currentBeat - lastBeat
                lastBeat = currentBeat

                rawBPM = 0
                if(beatDuration > 0):
                    rawBPM = 60000.0 / beatDuration
                
                if(self.debug == True): 
                    print("Raw BRM {}".format(rawBPM))

                # //This method sometimes glitches, it's better to go through whole moving average everytime
                # //IT's a neat idea to optimize the amount of work for moving avg. but while placing, removing finger it can screw up
                # //valuesBPMSum -= valuesBPM[bpmIndex];
                # //valuesBPM[bpmIndex] = rawBPM;
                # //valuesBPMSum += valuesBPM[bpmIndex];

                self.valuesBPM[self.bpmIndex] = rawBPM
                self.valuesBPMSum = 0
                i = 0
                while(i < self.PULSE_BPM_SAMPLE_SIZE):
                    self.valuesBPMSum = self.valuesBPMSum + self.valuesBPM[i]
                    i = i + 1

                if(self.debug == True): 
                    print("CurrentMoving Avg: {}".format(self.valuesBPM))

                self.bpmIndex = self.bpmIndex + 1
                self.bpmIndex = self.bpmIndex % self.PULSE_BPM_SAMPLE_SIZE

                if(self.valuesBPMCount < self.PULSE_BPM_SAMPLE_SIZE):
                    self.valuesBPMCount = self.valuesBPMCount + 1

                self.currentBPM = self.valuesBPMSum / self.valuesBPMCount
                if(self.debug == True): 
                    print("AVg. BPM: {}".format(self.currentBPM))

                self.currentPulseDetectorState = self.PULSE_TRACE_DOWN

                return True

        elif(self.currentPulseDetectorState == self.PULSE_TRACE_DOWN):
            if(sensor_value < prev_sensor_value):
                values_went_down = values_went_down + 1

            if(sensor_value < self.PULSE_MIN_THRESHOLD):
                self.currentPulseDetectorState = self.PULSE_IDLE;

        prev_sensor_value = sensor_value;

        return False;



    @property
    def red(self):
        return self.buffer_red[-1] if self.buffer_red else None

    @property
    def ir(self):
        return self.buffer_ir[-1] if self.buffer_ir else None

    def set_led_current(self, led_current_red=11.0, led_current_ir=11.0):
        # Validate the settings, convert to bit values.
        led_current_red = _get_valid(LED_CURRENT, led_current_red)
        led_current_ir = _get_valid(LED_CURRENT, led_current_ir)
        self.i2c.write_byte_data(I2C_ADDRESS, LED_CONFIG, (led_current_red << 4) | led_current_ir)

    def set_mode(self, mode):
        reg = self.i2c.read_byte_data(I2C_ADDRESS, MODE_CONFIG)
        self.i2c.write_byte_data(I2C_ADDRESS, MODE_CONFIG, reg & 0x74) # mask the SHDN bit
        self.i2c.write_byte_data(I2C_ADDRESS, MODE_CONFIG, reg | mode)

    def set_spo_config(self, sample_rate=100, pulse_width=1600):
        reg = self.i2c.read_byte_data(I2C_ADDRESS, SPO2_CONFIG)
        reg = reg & 0xFC  # Set LED pulsewidth to 00
        self.i2c.write_byte_data(I2C_ADDRESS, SPO2_CONFIG, reg | pulse_width)

    def enable_spo2(self):
        self.set_mode(MODE_SPO2)

    def disable_spo2(self):
        self.set_mode(MODE_HR)

    def enable_interrupt(self, interrupt_type):
        self.i2c.write_byte_data(I2C_ADDRESS, INT_ENABLE, (interrupt_type + 1)<<4)
        self.i2c.read_byte_data(I2C_ADDRESS, INT_STATUS)

    def get_number_of_samples(self):
        write_ptr = self.i2c.read_byte_data(I2C_ADDRESS, FIFO_WR_PTR)
        read_ptr = self.i2c.read_byte_data(I2C_ADDRESS, FIFO_RD_PTR)
        return abs(16+write_ptr - read_ptr) % 16

    def read_sensor(self):
        bytes = self.i2c.read_i2c_block_data(I2C_ADDRESS, FIFO_DATA, 4)
        # Add latest values.
        self.buffer_ir.append(bytes[0]<<8 | bytes[1])
        self.buffer_red.append(bytes[2]<<8 | bytes[3])
        # Crop our local FIFO buffer to length.
        self.buffer_red = self.buffer_red[-self.max_buffer_len:]
        self.buffer_ir = self.buffer_ir[-self.max_buffer_len:]

    def shutdown(self):
        reg = self.i2c.read_byte_data(I2C_ADDRESS, MODE_CONFIG)
        self.i2c.write_byte_data(I2C_ADDRESS, MODE_CONFIG, reg | 0x80)

    def reset(self):
        reg = self.i2c.read_byte_data(I2C_ADDRESS, MODE_CONFIG)
        self.i2c.write_byte_data(I2C_ADDRESS, MODE_CONFIG, reg | 0x40)

    def refresh_temperature(self):
        reg = self.i2c.read_byte_data(I2C_ADDRESS, MODE_CONFIG)
        self.i2c.write_byte_data(I2C_ADDRESS, MODE_CONFIG, reg | (1 << 3))

    def get_temperature(self):
        intg = _twos_complement(self.i2c.read_byte_data(I2C_ADDRESS, TEMP_INTG))
        frac = self.i2c.read_byte_data(I2C_ADDRESS, TEMP_FRAC)
        return intg + (frac * 0.0625)

    def get_rev_id(self):
        return self.i2c.read_byte_data(I2C_ADDRESS, REV_ID)

    def get_part_id(self):
        return self.i2c.read_byte_data(I2C_ADDRESS, PART_ID)

    def get_registers(self):
        return {
            "INT_STATUS": self.i2c.read_byte_data(I2C_ADDRESS, INT_STATUS),
            "INT_ENABLE": self.i2c.read_byte_data(I2C_ADDRESS, INT_ENABLE),
            "FIFO_WR_PTR": self.i2c.read_byte_data(I2C_ADDRESS, FIFO_WR_PTR),
            "OVRFLOW_CTR": self.i2c.read_byte_data(I2C_ADDRESS, OVRFLOW_CTR),
            "FIFO_RD_PTR": self.i2c.read_byte_data(I2C_ADDRESS, FIFO_RD_PTR),
            "FIFO_DATA": self.i2c.read_byte_data(I2C_ADDRESS, FIFO_DATA),
            "MODE_CONFIG": self.i2c.read_byte_data(I2C_ADDRESS, MODE_CONFIG),
            "SPO2_CONFIG": self.i2c.read_byte_data(I2C_ADDRESS, SPO2_CONFIG),
            "LED_CONFIG": self.i2c.read_byte_data(I2C_ADDRESS, LED_CONFIG),
            "TEMP_INTG": self.i2c.read_byte_data(I2C_ADDRESS, TEMP_INTG),
            "TEMP_FRAC": self.i2c.read_byte_data(I2C_ADDRESS, TEMP_FRAC),
            "REV_ID": self.i2c.read_byte_data(I2C_ADDRESS, REV_ID),
            "PART_ID": self.i2c.read_byte_data(I2C_ADDRESS, PART_ID),
        }
