from machine import Pin, PWM, freq
from rp2 import asm_pio, StateMachine, PIO

DEBUG = False  # Set to True to enable debug messages


PWM_OUTPUT_PIN_ABSOLUTE = 0  # Pin to generate the PWM signal, connect this pin to the INPUT_PULSE_PIN_ABSOLUTE pin
INPUT_PULSE_PIN_ABSOLUTE = 2  # Pin to measure the frequency of the PWM signal
TIMING_PULSE_PIN_ABSOLUTE = 3  # Pin to generate the timing pulses
SIDESET_PIN_ABSOLUTE = 1  # Pin to set the side-set pin

CPU_DEFAULT_FREQUENCY = 125_000_000  # 125 MHz
CPU_TARGET_FREQUENCY = 125_000_000  # 125 MHz
TIMING_PULSE_RATIO = 0x8  # 8 timing pulses for 1 gate time
TIMING_PULSE_FREQUENCY = 8  # 8 Hz

TIMING_PULSE_SM_ID = 0
PULSE_COUNTER_SM_ID = 1


# PIO program to count pulses, the gate time is controlled a side-set pin set by another PIO program
#! We now count BOTH the rising and falling edges of the input pulse, so the count will be doubled the actual frequency, but this allow us to no longer rely on wait, which could stall the program depending on the final state of the input pulse
@asm_pio(autopull=False, out_shiftdir=PIO.SHIFT_RIGHT)
def pulse_counter_pio(sideset_pin=SIDESET_PIN_ABSOLUTE):
    label("start")
    set(x, 0)  # Set the x register to 0, this is the counter register for the pulses
    set(y, 0)

    wait(1, gpio, sideset_pin)
    wait(0, gpio, sideset_pin)  # wait for the side-set pin to go low

    # start counting the pulses
    label("count")
    # save the current value of x to the ISR (!used for output to the FIFO!)
    mov(isr, x)
    mov(x, y)  # move the previous pin value from y to x
    set(y, 0)

    # unfortunately, "mov dest, pins" shift all 32 pins states, so we need to do some bit shifting to get the one pin value we want
    mov(osr, pins)  # move the current pin value to the OSR
    # shift the previous pin value bits to the y register. Let say we are measuring on pulse_pin 2, we will shift the first two bits, which is pin 0 and pin 1 states into the y register, then clear the y register, shift one more bit, and the y register will contain the state of pin 2
    out(y, 1)

    jmp(x_not_y, "increment")  # If the pin value has changed, jump to increment
    mov(x, isr)
    jmp("count")  # If the pin value has not changed, continue counting

    label("increment")
    mov(x, isr)  # Restore the previous value of x from the ISR
    jmp(x_dec, "check_side_set")  # Decrement x and jump to check_x
    label("check_side_set")  # Check the side-set pin
    jmp(pin, "push")  # If side-set is high, jump to push
    jmp("count")  # If side-set is still low, continue count

    label("push")  # Push the counter value to the FIFO
    mov(isr, x)  # Move the x register to the ISR
    push(noblock)  # Push the ISR to the FIFO
    jmp("start")  # Restart the program


# A second pio program to set a side-set pin, initialize the side-set pin to high
@asm_pio(sideset_init=PIO.OUT_HIGH)
def timing_pulse_pio(
    sm_id=TIMING_PULSE_SM_ID,
    pulse_pin=INPUT_PULSE_PIN_ABSOLUTE,
    timing_pin=TIMING_PULSE_PIN_ABSOLUTE,
    sideset_pin=SIDESET_PIN_ABSOLUTE,
    timing_ratio=TIMING_PULSE_RATIO,
):
    label("start")
    set(x, timing_ratio)  # Set the x register to timing_ratio
    # we will wait for timing_ratio timing pulses before setting the side-set pin
    set(y, 0)  # set the y register to 0 to comparison with the x register

    wait(1, pin, 0)
    wait(0, pin, 0)  # wait for the timing pulse to go low to synchronize the timing
    # set the side-set pin to 0 to let the pulse counter know that the gate time has started
    nop().side(0)

    label("loop")
    wait(1, pin, 0)  # Wait for high pulse on input pin
    wait(0, pin, 0)  # Wait for low pulse on input pin

    # for debugging purposes, move the x to the isr
    mov(isr, x)  # Move the x register to the ISR
    push(noblock)  # push the isr to the fifo

    # One pulse has been received, decrement the x register
    jmp(x_dec, "check_x")  # Decrement x and jump to check_x
    label("check_x")
    jmp(x_not_y, "loop")  # if x is not equal to y, jump back to the loop
    jmp("start").side(1)  # else set the side-set pin to 1 and jump back to the start


# Class to handle the pulse counting
class PulseCounter:
    def __init__(
        self,
        pulse_counter_pio_program,
        timing_pulse_pio_program,
        pulse_counter_pio_sm_id: int,
        timing_pulse_pio_sm_id: int,
        input_pulse_pin: int,
        timing_pulse_pin: int,
        timing_pulse_frequency,
        timing_pulse_ratio,
        sideset_pulse_pin: int,
        freq,
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

        :return: None
        """
        # state machine id for the pulse counter
        self.pulse_counter_pio_sm_id = pulse_counter_pio_sm_id
        # state machine id for the timing pulse
        self.timing_pulse_pio_sm_id = timing_pulse_pio_sm_id
        # input pin for the waveform to be measured
        self.pulse_pin = Pin(input_pulse_pin, Pin.IN)
        # timing pulse pin to gate the frequency measurement
        self.timing_pin = Pin(timing_pulse_pin, Pin.OUT)
        self.timing_pin_pwm = PWM(self.timing_pin)
        self.timing_pin_pwm.freq(timing_pulse_frequency)
        self.timing_pin_pwm.duty_u16(32768)  # Set duty cycle to 50%
        self.timing_interval_ms = 1000 / timing_pulse_frequency * timing_pulse_ratio
        # sideset pin to control the gate time
        self.sideset_pin = Pin(sideset_pulse_pin, Pin.OUT)
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
        if DEBUG:
            print(
                f"Pulse Counter State Machine ID: {self.pulse_counter_pio_sm_id}, Timing Pulse State Machine ID: {self.timing_pulse_pio_sm_id}, Timing Pulse Frequency: {timing_pulse_frequency} Hz, Timing Interval: {self.timing_interval_ms} ms"
            )

    def read_pulse_count(self):
        if self.pulse_counter_pio_sm.rx_fifo() == 0:
            return -1
        else:
            pulse_count = self.pulse_counter_pio_sm.get()  # Get value from the FIFO
            return (0x100000000 - pulse_count) & 0xFFFFFFFF  # flip the value

    def read_timing_count(self):  # for debugging purposes
        if self.timing_pulse_pio_sm.rx_fifo() == 0:
            return -1
        else:
            return self.timing_pulse_pio_sm.get()

    def reset(self):
        self.pulse_counter_pio_sm.restart()
        self.timing_pulse_pio_sm.restart()

    def start(self):
        self.timing_pulse_pio_sm.active(1)
        self.pulse_counter_pio_sm.active(1)

    def stop(self):
        self.pulse_counter_pio_sm.active(0)
        self.timing_pulse_pio_sm.active(0)


def main():
    try:
        # clear state machines
        StateMachine(TIMING_PULSE_SM_ID).active(0)
        StateMachine(PULSE_COUNTER_SM_ID).active(0)

        freq(CPU_TARGET_FREQUENCY)  # Set the CPU frequency
        print(f"cpu frequency set to: {freq() / 1_000_000} MHz")

        # Generate test PWM signal on PWM_OUTPUT_PIN
        pwm_test_signal = PWM(Pin(PWM_OUTPUT_PIN_ABSOLUTE, Pin.OUT))
        pwm_test_signal.freq(2500000)  # Set the frequency of the PWM signal
        # pwm_test_signal.freq(20)  # Set the frequency of the PWM signal
        pwm_test_signal.duty_u16(32768)  # Set duty cycle to 50%

        # Initialize the pulse counter for the timer method
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
        pulse_counter.reset()
        pulse_counter.start()

        # print the timing pulse count
        while True:
            timing_pulse_count = pulse_counter.read_timing_count()
            if timing_pulse_count == 1:
                pulse_count = pulse_counter.read_pulse_count()
                frequency = pulse_count / timing_interval_ms * 1000
                if (
                    pulse_count > 1_000_000
                ):  # change to MHz if the frequency is too high
                    print(
                        f"Generated PWM Frequency: {pwm_test_signal.freq() / 1_000_000} MHz, Gate Time: {timing_interval_ms} ms, PIO raw count: {pulse_count}, Frequency: {frequency / 1_000_000} MHz"
                    )
                elif pulse_count > 1000:  # change to kHz if the frequency is too high
                    print(
                        f"Generated PWM Frequency: {pwm_test_signal.freq() / 1000} kHz, Gate Time: {timing_interval_ms} ms, PIO raw count: {pulse_count}, Frequency: {frequency / 1000} kHz"
                    )
                else:
                    print(
                        f"Generated PWM Frequency: {pwm_test_signal.freq()} Hz, Gate Time: {timing_interval_ms} ms, PIO raw count: {pulse_count}, Frequency: {frequency} Hz"
                    )
            elif timing_pulse_count == -1:
                continue
            else:
                print(f"Timing Pulse Count: {timing_pulse_count}")
                continue

    except KeyboardInterrupt:
        pulse_counter.stop()
        print("Stopped the pulse counter")
        print("Exiting the program")


if __name__ == "__main__":
    main()
