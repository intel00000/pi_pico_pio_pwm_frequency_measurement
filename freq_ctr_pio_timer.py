from machine import Pin, Timer, PWM, freq
from rp2 import asm_pio, StateMachine
import utime


PWM_TESTING_PIN_ABSOLUTE = 0  # Testing PWM signal, connect this to the INPUT_PULSE_PIN
PWM_TESTING_FREQUENCY = 10  # Frequency of the testing PWM signal

CPU_TARGET_FREQUENCY = 125_000_000  # Target CPU frequency in Hz
CPU_DEFAULT_FREQUENCY = 125_000_000  # 125 MHz

PWM_OUTPUT_PIN = 0
PULSE_INPUT_PIN = 2


# PIO program to count pulses
@asm_pio()
def pulse_counter_pio():
    set(x, 0)  # Reset counter
    wrap_target()
    label("count")
    # we will count the falling edge of the pulse
    wait(1, pin, 0)  # Wait for high pulse
    wait(0, pin, 0)  # Wait for low pulse
    jmp(x_dec, "count")  # Decrement counter
    wrap()


# Class to handle the pulse counting
class PulseCounter:
    def __init__(self, sm_id, pin, program):
        self.sm_id = sm_id
        self.pin = pin
        # Set the frequency to match the system clock
        self.sm = StateMachine(
            sm_id, program, freq=freq(), in_base=Pin(pin, Pin.IN, Pin.PULL_UP)
        )
        self.sm.active(1)
        self.counter = 0

    def read(self):
        self.sm.exec("mov(isr, x)")
        self.sm.exec("push()")
        self.counter = self.sm.get()
        return 0xFFFFFFFF - self.counter

    def reset(self):
        self.sm.exec("set(x, 0)")  # Reset the counter

    def __del__(self):
        self.sm.active(0)


# Initialize the pulse counter for the timer method
pulse_counter_pio = PulseCounter(0, PULSE_INPUT_PIN, pulse_counter_pio)
pulse_frequency_pio = 0

time_us = utime.ticks_us()


# Timer callback to read and reset the counter
def timer_callback_timer(timer):
    global pulse_counter_pio, pulse_frequency_pio, time_us
    # adjust the frequency based on the elapsed utime
    pulse_frequency_pio = pulse_counter_pio.read() / (utime.ticks_us() - time_us) * 1e6
    pulse_counter_pio.reset()
    time_us = utime.ticks_us()


def main():
    try:
        freq(CPU_TARGET_FREQUENCY)  # Set the CPU frequency
        print(f"CPU freq set to: {freq() / 1_000_000} MHz")

        # Generate testing PWM signal
        pwm_testing = PWM(Pin(PWM_TESTING_PIN_ABSOLUTE, Pin.OUT))
        pwm_testing.init(freq=PWM_TESTING_FREQUENCY, duty_u16=32768)

        # Set up the timer to periodically read the counter
        timer = Timer(period=1000, mode=Timer.PERIODIC, callback=timer_callback_timer)

        previous_time = utime.ticks_us()
        while True:
            # do some math here to simulate work
            for i in range(1000):
                i = i + i
            current_time = utime.ticks_us()
            tick_diff = utime.ticks_diff(current_time, previous_time)
            if tick_diff >= 1e6:  # 1 second
                if pulse_frequency_pio > 1_000_000:  # MHz
                    freq_str = f"{pulse_frequency_pio / 1_000_000} MHz"
                    gen_freq_str = f"{pwm_testing.freq() / 1_000_000} MHz"
                elif pulse_frequency_pio > 1_000:
                    freq_str = f"{pulse_frequency_pio / 1_000} kHz"
                    gen_freq_str = f"{pwm_testing.freq() / 1_000} kHz"
                else:
                    freq_str = f"{pulse_frequency_pio} Hz"
                    gen_freq_str = f"{pwm_testing.freq()} Hz"
                print(
                    f"Tick elapsed: {tick_diff} us, Generated PWM Frequency: {gen_freq_str}, Measured Frequency: {freq_str}"
                )
                previous_time = current_time

    except KeyboardInterrupt:
        global pulse_counter_pio
        timer.deinit()
        pwm_testing.deinit()
        del pulse_counter_pio


if __name__ == "__main__":
    main()
