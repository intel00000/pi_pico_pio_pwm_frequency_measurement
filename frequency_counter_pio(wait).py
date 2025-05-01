from machine import Pin, PWM, freq
from rp2 import asm_pio, StateMachine, PIO
import gc

# Pin to generate the PWM signal, connect this pin to the INPUT_PULSE_PIN_ABSOLUTE pin
PWM_OUTPUT_PIN_ABSOLUTE = 0
PWM_OUTPUT_PIN = Pin(PWM_OUTPUT_PIN_ABSOLUTE, Pin.OUT)
# Pin to measure the frequency of the PWM signal
INPUT_PULSE_PIN_ABSOLUTE = 2
INPUT_PULSE_PIN = Pin(INPUT_PULSE_PIN_ABSOLUTE, Pin.IN, Pin.PULL_DOWN)
# Pin to generate the timing pulses
TIMING_PULSE_PIN_ABSOLUTE = 3
TIMING_PULSE_PIN = Pin(TIMING_PULSE_PIN_ABSOLUTE, Pin.OUT)
TIMING_PULSE_PIN_PWM = PWM(TIMING_PULSE_PIN_ABSOLUTE)
# Pin to set the side-set pin
SIDESET_PIN_ABSOLUTE = 1
SIDESET_PIN = Pin(SIDESET_PIN_ABSOLUTE, Pin.OUT)

CPU_DEFAULT_FREQUENCY = 125_000_000  # 125 MHz
CPU_TARGET_FREQUENCY = 125_000_000  # 125 MHz
TIMING_PULSE_RATIO = 0x8  # 8 timing pulses for 1 gate time
TIMING_PULSE_FREQUENCY = 8  # 8 Hz

TIMING_PULSE_SM_ID = 6
PULSE_COUNTER_SM_ID = 7


# PIO program to count pulses, the gate time is controlled a side-set pin set by another PIO program
@asm_pio()
def pulse_counter_pio(
    pulse_pin=INPUT_PULSE_PIN_ABSOLUTE,
    timing_pin=TIMING_PULSE_PIN_ABSOLUTE,
    sideset_pin=SIDESET_PIN_ABSOLUTE,
):
    label("start")
    set(x, 0)  # Set the x register to 0, this is the counter register for the pulses

    wait(1, gpio, sideset_pin)
    wait(0, gpio, sideset_pin)  # wait for the side-set pin to go low

    # start counting the pulses
    label("count")
    wait(0, pin, 0)  # Wait for low pulse on input pin
    wait(1, pin, 0)  # Wait for high pulse on input pin
    wait(0, pin, 0)  # Wait for low pulse on input pin
    jmp(x_dec, "check_sideset")  # Decrement counter and jump to check_pin
    label("check_sideset")
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

    wait(0, gpio, pulse_pin)
    wait(1, gpio, pulse_pin)  # synchronize the timing with the actual pulse
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
        pulse_counter_pio_sm_id=PULSE_COUNTER_SM_ID,
        timing_pulse_pio_sm_id=TIMING_PULSE_SM_ID,
        pulse_pin=INPUT_PULSE_PIN,
        timing_pin=TIMING_PULSE_PIN,
        sideset_pin=SIDESET_PIN,
    ):
        # state machine id for the pulse counter
        self.pulse_counter_pio_sm_id = pulse_counter_pio_sm_id
        # state machine id for the timing pulse
        self.timing_pulse_pio_sm_id = timing_pulse_pio_sm_id
        # input pin for the waveform to be measured
        self.pulse_pin = pulse_pin
        # timing pulse pin to measure the frequency of the input
        self.timing_pin = timing_pin
        # sideset pin to control the gate time
        self.sideset_pin = sideset_pin
        # setup the pulse counter state machine
        self.pulse_counter_pio_sm = StateMachine(
            self.pulse_counter_pio_sm_id,
            pulse_counter_pio_program,
            freq=freq(),
            in_base=self.pulse_pin,
            jmp_pin=self.sideset_pin,
            sideset_base=self.sideset_pin,
        )
        # setup the timing pulse state machine
        self.timing_pulse_pio_sm = StateMachine(
            self.timing_pulse_pio_sm_id,
            timing_pulse_pio_program,
            freq=freq(),
            in_base=self.timing_pin,
            sideset_base=self.sideset_pin,
        )
        print(
            f"Pulse Counter State Machine ID: {self.pulse_counter_pio_sm_id}, Timing Pulse State Machine ID: {self.timing_pulse_pio_sm_id}"
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


def timing_pulse_generator(
    timing_pulse=TIMING_PULSE_PIN_PWM,
    timing_pulse_frequency=TIMING_PULSE_FREQUENCY,
    timing_pulse_ratio=TIMING_PULSE_RATIO,
):
    timing_pulse.freq(timing_pulse_frequency)
    timing_pulse.duty_u16(32768)
    timing_interval_ms = 1000 / timing_pulse_frequency * timing_pulse_ratio
    print(
        f"Timing Pulse Frequency: {timing_pulse_frequency} Hz, Timing Interval: {timing_interval_ms} ms"
    )
    return timing_interval_ms


def main():
    # run a gc.collect() to free up memory
    gc.collect()
    try:
        # clear all state machines
        StateMachine(TIMING_PULSE_SM_ID).active(0)
        StateMachine(PULSE_COUNTER_SM_ID).active(0)

        freq(CPU_TARGET_FREQUENCY)  # Set the CPU frequency
        print(f"cpu frequency set to: {freq() / 1_000_000} MHz")

        # Generate test PWM signal on PWM_OUTPUT_PIN
        pwm_test_signal = PWM(PWM_OUTPUT_PIN)
        pwm_test_signal.freq(2500000)  # Set the frequency of the PWM signal
        # pwm_test_signal.freq(20)  # Set the frequency of the PWM signal
        pwm_test_signal.duty_u16(32768)  # Set duty cycle to 50%

        # Generate timing pulses
        timing_interval_ms = timing_pulse_generator()

        # Initialize the pulse counter for the timer method
        pulse_counter = PulseCounter(
            pulse_counter_pio_program=pulse_counter_pio,
            timing_pulse_pio_program=timing_pulse_pio,
        )

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
                # print(f"Timing Pulse Count: {timing_pulse_count}")
                continue

    except KeyboardInterrupt:
        pulse_counter.stop()
        print("Stopped the pulse counter")
        print("Exiting the program")


if __name__ == "__main__":
    main()
