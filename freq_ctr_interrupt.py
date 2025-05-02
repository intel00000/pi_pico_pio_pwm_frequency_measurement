from machine import Pin, PWM, Timer
import utime
import gc

PWM_TESTING_FREQUENCY = 20_000

PWM_OUTPUT_PIN = 0
PULSE_INPUT_PIN = 2


# Class to handle pulse counting using interrupts
class PulseCounterInterrupt:
    def __init__(self, pin):
        self.pin = pin
        self.last_time = utime.ticks_us()

        self.counter = 0
        self.pin.irq(trigger=Pin.IRQ_FALLING, handler=self.callback)

    def callback(self, pin):
        # we only record time every 10 pulses to reduce the overhead
        self.counter += 1

    def get_frequency(self):
        last_time = self.last_time
        counter = self.counter
        self.counter = 0

        current_time = utime.ticks_us()
        self.last_time = current_time
        time_diff = utime.ticks_diff(current_time, last_time)  # in microseconds

        if time_diff > 0:
            return counter * 1e6 / time_diff  # Convert to Hz
        else:
            return 0.0

    def deinit(self):
        self.pin.irq(handler=None)


# Initialize the pulse counter
pulse_counter_interrupt = PulseCounterInterrupt(Pin(PULSE_INPUT_PIN, Pin.IN))


def main():
    global pulse_counter_interrupt

    # Generate testing PWM signal
    pwm_testing = PWM(Pin(PWM_OUTPUT_PIN, Pin.OUT))
    pwm_testing.init(freq=PWM_TESTING_FREQUENCY, duty_u16=32768)

    previous_time = utime.ticks_us()
    while True:
        # do some math here to simulate work
        for i in range(1000):
            i = i + i
        current_time = utime.ticks_us()
        tick_diff = utime.ticks_diff(current_time, previous_time)
        if tick_diff >= 1e6:  # 1 second
            print(
                f"Tick elapsed: {tick_diff} us, PWM Frequency: {pwm_testing.freq()} Hz, Measured Frequency: {pulse_counter_interrupt.get_frequency()} Hz"
            )
            previous_time = current_time
            gc.collect()


if __name__ == "__main__":
    main()
