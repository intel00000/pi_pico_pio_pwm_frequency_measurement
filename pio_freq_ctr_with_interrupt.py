from machine import Pin, PWM, freq
from rp2 import asm_pio, StateMachine, PIO

DEBUG = False  # Set to True to enable debug messages

PWM_TESTING_PIN_ABSOLUTE = 0  # Testing PWM signal, connect this to the INPUT_PULSE_PIN
PWM_TESTING_PIN_ABSOLUTE_FREQUENCY = 10_000_000  # Frequency of the testing PWM signal

INPUT_PULSE_PIN_ABSOLUTE = 2  # Pin to measure the frequency of the PWM signal
TIMING_PULSE_PIN_ABSOLUTE = 6  # Pin to generate the timing pulses
SIDESET_PIN_ABSOLUTE = 1  # Pin to set the side-set pin

CPU_TARGET_FREQUENCY = 200_000_000  # Target CPU frequency in Hz
CPU_DEFAULT_FREQUENCY = 125_000_000  # 125 MHz

# !change this if you to change the measurement frequency
# lowest PWM frequency under the target CPU frequency
TIMING_PULSE_FREQUENCY = int(8 * CPU_TARGET_FREQUENCY / CPU_DEFAULT_FREQUENCY)
# divided the timing pulse frequency by this value to get the true gate time
TIMING_PULSE_RATIO = int(8 * CPU_TARGET_FREQUENCY / CPU_DEFAULT_FREQUENCY)

TIMING_PULSE_SM_ID = 0
PULSE_COUNTER_SM_ID = 4


# PIO program to count pulses, the gate time is controlled a side-set pin set by another PIO program
# !# we now compared the previous pin state with the current pin state, this allow us to no longer rely on wait, which could stall the program if the final state of the input pulse are fixed at high (then the pio program will forever wait for low), this also allow the pio program to constantly output the pulse count to the FIFO.
@asm_pio(autopull=False, out_shiftdir=PIO.SHIFT_RIGHT)
def pulse_counter_pio(sideset_pin=SIDESET_PIN_ABSOLUTE):
    # Reset registers to 0
    set(x, 0)
    set(y, 0)
    # wait for the side-set pin to go low
    wait(1, gpio, sideset_pin)
    wait(0, gpio, sideset_pin)

    # start counting the pulses
    label("count")
    mov(isr, x)  # temporarily save the current value of x to the ISR
    mov(x, y)  # move the previous pin value from y to x
    # set(y, 0)  # doesn't seem to matter

    # "mov dest, pins" shift all 32 pins states, so we need to do some bit shifting to get the one pin value we want, luckily the pin we want are the in_base pins which is the last bit of all 32 bits.
    # Noted the last bit of "mov dest, pins" start with the in_base pin, then increment and wrap around to the pin before the in_base pin, so if one change the in_base, the return also change.
    mov(osr, pins)  # move all pin states to the OSR

    # shift the last bit of the OSR to the y register, this is the one pin state we want to check
    #! MUST set out_shiftdir=PIO.SHIFT_RIGHT in the @asm_pio decorator to shift the OSR to the right, default is PIO.SHIFT_LEFT
    out(y, 1)

    jmp(x_not_y, "check_falling_edge")  # If the pin value changed, check_falling_edge

    label("restore_x")  # Restore the previous value of x, continue counting
    mov(x, isr)
    jmp("count")

    label("check_falling_edge")
    jmp(not_y, "increment")  # If the current pin state is low, jump to increment
    jmp(
        "restore_x"
    )  # If the current pin state is high, restore x and continue counting

    label("increment")
    mov(x, isr)  # Restore x
    jmp(x_dec, "check_side_set")  # Decrement x and check the side-set pin
    label("check_side_set")  # Check the side-set pin
    jmp(pin, "push")  # If side-set is high, jump to push
    jmp("count")  # If side-set is still low, continue count

    label("push")  # Push the counter value to the FIFO
    mov(isr, x)
    push(noblock)  # won't wait for the FIFO to become available


# A second pio program to set a side-set pin, initialize the side-set pin to high
@asm_pio(autopull=False, sideset_init=PIO.OUT_HIGH)
def timing_pulse_pio(irq_id=TIMING_PULSE_SM_ID, pulse_pin=INPUT_PULSE_PIN_ABSOLUTE):
    # here we refer to the PWM example in RP2040 datasheet and do a noblock pull. if nothing on the TX FIFO, this will copy X to OSR, if there is something on the TX FIFO, this will copy the value from the TX FIFO to OSR and then pull it to the RX FIFO.
    # this allow us to change the timing ratio on the fly
    pull(noblock)
    mov(x, osr)
    mov(osr, x)

    # synchronize the timing with the actual pulse
    wait(1, gpio, pulse_pin)
    wait(0, gpio, pulse_pin)
    irq(rel(irq_id))  # raise a interrupt to reinitialize the timing pulse
    # synchronize to reference pulse
    wait(1, pin, 0)
    wait(0, pin, 0)
    nop().side(0)  # set side-set pin

    label("loop")
    wait(1, pin, 0)  # Wait for high pulse on input pin
    wait(0, pin, 0)  # Wait for low pulse on input pin

    # for debugging purposes, move the x to the isr
    mov(isr, x)  # Move the x register to the ISR
    push(noblock)  # push the isr to the fifo

    # One pulse has been received, decrement the x register, check if it is zero
    jmp(x_dec, "loop")
    mov(x, osr).side(1)  # else set the side-set pin to 1 and go back to the start


# Class to handle the pulse counting
class PulseCounter:
    def __init__(
        self,
        pulse_counter_pio_program: PIO,
        timing_pulse_pio_program: PIO,
        pulse_counter_pio_sm_id: int,
        timing_pulse_pio_sm_id: int,
        input_pulse_pin: int,
        timing_pulse_pin: int,
        timing_pulse_frequency: int,
        timing_pulse_ratio: int,
        sideset_pulse_pin: int,
        freq: int,
    ):
        """
        Initialize the PulseCounter class.

        :param pulse_counter_pio_program: PIO program to count pulses
        :param timing_pulse_pio_program: PIO program to generate timing pulses
        :param pulse_counter_pio_sm_id: State machine ID for the pulse counter
        :param timing_pulse_pio_sm_id: State machine ID for the timing pulse

        :param input_pulse_pin: Pin to measure the frequency of the input pulse
        :param timing_pulse_pin: Pin to generate the timing pulses
        :param timing_pulse_frequency: Frequency of the timing pulse
        :param timing_pulse_ratio: Ratio of the timing pulse to the gate time
        :param sideset_pulse_pin: Pin to control the gate time (sideset pin)
        :param freq: Frequency of the PIO state machines

        :return: None
        """
        # the pulse counter sm id
        self.pulse_counter_pio_sm_id = pulse_counter_pio_sm_id
        # state machine id for the timing pulse
        self.timing_pulse_pio_sm_id = timing_pulse_pio_sm_id
        # input pin for the waveform to be measured
        self.pulse_pin = Pin(input_pulse_pin, Pin.IN)
        # reference timing pulse pin to gate the frequency measurement
        self.timing_pin = Pin(timing_pulse_pin, Pin.OUT)
        self.timing_pin_pwm = PWM(self.timing_pin)
        self.timing_pulse_ratio = timing_pulse_ratio
        self.timing_pulse_frequency = timing_pulse_frequency
        self.timing_interval_ms = 1000 / timing_pulse_frequency * timing_pulse_ratio
        self.timing_pin_pwm.init(freq=self.timing_pulse_frequency, duty_u16=32768)
        # sideset pin to control the gate time
        self.sideset_pin = Pin(sideset_pulse_pin, Pin.OUT, value=1)
        # clear state machines
        StateMachine(self.pulse_counter_pio_sm_id).active(0)
        StateMachine(self.timing_pulse_pio_sm_id).active(0)
        # setup the pulse counter state machine
        self.pulse_counter_pio_sm = StateMachine(
            self.pulse_counter_pio_sm_id,
            pulse_counter_pio_program,
            freq=freq,
            in_base=self.pulse_pin,
            jmp_pin=self.sideset_pin,
            sideset_base=self.sideset_pin,
        )
        # setup the timing pulse state machine
        self.timing_pulse_pio_sm = StateMachine(
            self.timing_pulse_pio_sm_id,
            timing_pulse_pio_program,
            freq=freq,
            in_base=self.timing_pin,
            set_base=self.timing_pin,
            sideset_base=self.sideset_pin,
        )
        self.set_timing_pulse_ratio(timing_pulse_ratio)
        self.timing_pulse_pio_sm_irq = self.timing_pulse_pio_sm.irq(
            handler=self.callback, trigger=1, hard=True
        )

        if DEBUG:
            print(
                f"Pulse Counter SM ID: {self.pulse_counter_pio_sm_id}, Timing Pulse SM ID: {self.timing_pulse_pio_sm_id}, Timing Pulse Frequency: {timing_pulse_frequency} Hz, Timing Interval: {self.timing_interval_ms} ms"
            )

    def callback(self, sm):
        """
        Callback function to reinitialize the timing pulse signal when an interrupt occurs.

        :param sm: The state machine that triggered the interrupt
        :return: None
        """
        self.timing_pin_pwm.deinit()
        self.timing_pin_pwm.init(freq=self.timing_pulse_frequency, duty_u16=32768)

    def set_timing_pulse_ratio(self, timing_pulse_ratio: int):
        """
        On the fly set the timing pulse ratio, this will change the gate time.

        :param timing_pulse_ratio: Ratio of the timing pulse to the gate time
        :return: None
        """
        if timing_pulse_ratio <= 1:
            raise ValueError("Timing pulse ratio must be greater than 1")
        self.timing_pulse_ratio = timing_pulse_ratio
        self.timing_pulse_pio_sm.put(self.timing_pulse_ratio - 1)

    def read_pulse_count(self):
        """
        Read the pulse count from the pulse counter state machine.
        Flips the value to get the correct count.

        :return: The pulse count, or -1 if the FIFO is empty
        """
        if self.pulse_counter_pio_sm.rx_fifo() == 0:
            return -1
        else:
            pulse_count = self.pulse_counter_pio_sm.get()  # Get value from the FIFO
            if DEBUG:
                # output the raw binary value of the pulse count
                binary_value = f"{pulse_count:032b}"
                print(f"Raw Pulse Count (before flip): {binary_value}")
            return (0x100000000 - pulse_count) & 0xFFFFFFFF  # flip the value

    def read_timing_count(self):  # for debugging purposes
        """
        Read the timing pulse count from the timing pulse state machine.

        This will return the number of timing pulses received since the last call to this method.
        If the FIFO is empty, it returns -1.

        :return: The timing pulse count, or -1 if the FIFO is empty
        """
        if self.timing_pulse_pio_sm.rx_fifo() == 0:
            return -1
        else:
            return self.timing_pulse_pio_sm.get()

    def restart(self):
        """
        Restart the pulse counter and timing pulse state machines.

        This will reset the state machines and set the timing pulse ratio.

        :return: None
        """
        self.pulse_counter_pio_sm.restart()
        self.pulse_counter_pio_sm.active(1)

        self.timing_pulse_pio_sm.active(1)
        self.timing_pulse_pio_sm.restart()
        self.set_timing_pulse_ratio(self.timing_pulse_ratio)

    def start(self):
        self.timing_pulse_pio_sm.active(1)
        self.pulse_counter_pio_sm.active(1)

    def stop(self):
        self.pulse_counter_pio_sm.active(0)

        self.set_timing_pulse_ratio(self.timing_pulse_ratio)
        self.timing_pulse_pio_sm.active(0)


def main():
    try:
        freq(CPU_TARGET_FREQUENCY)  # Set the CPU frequency
        print(f"CPU freq set to: {freq() / 1_000_000} MHz")

        # Generate test PWM signal on PWM_OUTPUT_PIN_ABSOLUTE pin
        pwm_test_signal = PWM(Pin(PWM_TESTING_PIN_ABSOLUTE, Pin.OUT))
        pwm_test_signal.init(freq=PWM_TESTING_PIN_ABSOLUTE_FREQUENCY, duty_u16=32768)

        # Initialize the pulse counter
        pulse_counter = PulseCounter(
            pulse_counter_pio_program=pulse_counter_pio,
            timing_pulse_pio_program=timing_pulse_pio,
            pulse_counter_pio_sm_id=PULSE_COUNTER_SM_ID,
            timing_pulse_pio_sm_id=TIMING_PULSE_SM_ID,
            input_pulse_pin=INPUT_PULSE_PIN_ABSOLUTE,
            timing_pulse_pin=TIMING_PULSE_PIN_ABSOLUTE,
            timing_pulse_frequency=TIMING_PULSE_FREQUENCY,
            timing_pulse_ratio=TIMING_PULSE_RATIO,
            sideset_pulse_pin=SIDESET_PIN_ABSOLUTE,
            freq=freq(),
        )
        timing_interval_ms = pulse_counter.timing_interval_ms

        # start the pulse counter
        pulse_counter.start()

        # print the timing pulse count
        while True:
            timing_pulse_count = pulse_counter.read_timing_count()
            if DEBUG and timing_pulse_count != -1:
                print(f"Timing Pulse Count: {timing_pulse_count}")

            if timing_pulse_count == 0:
                pulse_count = pulse_counter.read_pulse_count()
                frequency = pulse_count / timing_interval_ms * 1000
                if pulse_count > 1_000_000:  # MHz
                    gen_freq_str = f"{pwm_test_signal.freq() / 1_000_000} MHz"
                    freq_str = f"{frequency / 1_000_000} MHz"
                elif pulse_count > 1000:  # kHz
                    gen_freq_str = f"{pwm_test_signal.freq() / 1000} kHz"
                    freq_str = f"{frequency / 1000} kHz"
                else:
                    gen_freq_str = f"{pwm_test_signal.freq()} Hz"
                    freq_str = f"{frequency} Hz"
                print(
                    f"Generated PWM Frequency: {gen_freq_str}, Gate Time: {timing_interval_ms} ms, PIO raw count: {pulse_count}, Frequency: {freq_str}"
                )

    except KeyboardInterrupt:
        pulse_counter.stop()
        print("Stopped the pulse counter")
        print("Exiting the program")


if __name__ == "__main__":
    main()
