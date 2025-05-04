from machine import Pin, PWM, freq
from rp2 import asm_pio, StateMachine, PIO

DEBUG = False  # Set to True to enable debug messages

PWM_TESTING_PIN_ABSOLUTE = 0  # Testing PWM signal, connect this to the INPUT_PULSE_PIN
PWM_TESTING_PIN_ABSOLUTE_FREQUENCY = 10  # Frequency of the testing PWM signal

INPUT_PULSE_PIN_ABSOLUTE = 2  # Pin to measure the frequency of the PWM signal
TIMING_PULSE_PIN_ABSOLUTE = 6  # Pin to generate the timing pulses
SIDESET_PIN_ABSOLUTE = 1  # Pin to set the side-set pin

CPU_TARGET_FREQUENCY = 125_000_000  # Target CPU frequency in Hz
CPU_DEFAULT_FREQUENCY = 125_000_000  # 125 MHz

# !change this if you to change the measurement frequency
# lowest PWM frequency under the target CPU frequency
TIMING_PULSE_FREQUENCY = int(8 * CPU_TARGET_FREQUENCY / CPU_DEFAULT_FREQUENCY)
# divided the timing pulse frequency by this value to get the true gate time
TIMING_PULSE_RATIO = int(8 * CPU_TARGET_FREQUENCY / CPU_DEFAULT_FREQUENCY)

PWM_SM_ID = 2
TIMING_PULSE_SM_ID = 1
PULSE_COUNTER_SM_ID = 0


# PIO program to count pulses, the gate time is controlled a side-set pin set by another PIO program
# !# we now compared the previous pin state with the current pin state, this allow us to no longer rely on wait, which could stall the program if the final state of the input pulse are fixed at high (then the pio program will forever wait for low), this also allow the pio program to constantly output the pulse count to the FIFO.
@asm_pio(autopull=False, out_shiftdir=PIO.SHIFT_RIGHT)
def pulse_counter_pio(sideset_pin=SIDESET_PIN_ABSOLUTE):
    # Reset registers to 0
    set(x, 0)
    # wait for rising edge of the side-set pin
    wait(0, gpio, sideset_pin)
    wait(1, gpio, sideset_pin)

    # start counting the pulses
    label("count")
    mov(isr, x)  # temporarily save the current value of x to the ISR
    mov(x, y)  # move the previous pin value from y to x
    mov(osr, pins)  # move all pin states to the OSR
    out(y, 1)  # shift the last bit of the OSR to the y register
    jmp(x_not_y, "check_falling_edge")  # If the pin value changed, check_falling_edge

    label("restore_x")  # Restore the previous value of x, continue counting
    mov(x, isr)
    jmp("count")

    label("check_falling_edge")
    jmp(not_y, "increment")  # If the current pin state is low, jump to increment
    jmp("restore_x")  # If the state is high, restore x and continue counting

    label("increment")
    mov(x, isr)  # Restore x
    jmp(x_dec, "check_sideset")  # Decrement x and check the side-set pin
    label("check_sideset")  # Check the side-set pin
    jmp(pin, "count")  # If side-set is low, jump to push

    # Push the counter value to the FIFO
    mov(isr, x)
    push(noblock)  # won't wait for the FIFO to become available


# A second pio program to set a side-set pin, initialize the side-set pin to low
@asm_pio(sideset_init=PIO.OUT_LOW)
def timing_pulse_pio(pulse_pin=INPUT_PULSE_PIN_ABSOLUTE):
    wait(1, gpio, pulse_pin)  # synchronize to reference pulse
    wait(0, gpio, pulse_pin)

    wait(1, pin, 0)  # synchronize to reference pulse
    wait(0, pin, 0)

    label("loop")
    wait(1, pin, 0).side(1)  # Wait for falling edge on input pin, set side-set pin high
    wait(0, pin, 0)

    mov(isr, x)  # for debugging purposes, move the counter value to the ISR
    push(noblock)  # push the isr to the fifo

    jmp(x_dec, "loop")  # Decrement the x register, check if it is zero
    mov(x, y).side(0)  # else set the side-set pin to low and go back to the start


# A third pio program to generate the reference PWM pulse
# Adapted from the RP2040 datasheet PWM pio example.
@asm_pio(sideset_init=PIO.OUT_LOW)
def pwm_pio():
    mov(y, isr).side(0)  # Move the value from the ISR to the y register

    label("countloop")
    jmp(x_not_y, "noset")
    jmp("skip").side(1)  # Set the side-set pin to high if x == y

    label("noset")
    nop()  # Single dummy cycle to keep the two paths the same length

    label("skip")
    jmp(y_dec, "countloop")  # Decrement x and loop until x is zero


# prioritizes exact frequency over duty resolution
def compute_best_pwm_pio_parameters_fast(
    system_freq,
    target_pwm_freq,
    duty_percent,
    verbose=DEBUG,
    instr_per_loop=3,
    tolerance=1e-9,
):
    if not (0.0 <= duty_percent <= 100.0):
        raise ValueError("Duty cycle must be between 0 and 100")
    if target_pwm_freq <= 0 or system_freq <= 0:
        raise ValueError("Frequencies must be positive")

    best_config = None
    min_error = float("inf")

    for div_int in range(1, 65536):  # 16-bit int divider
        for div_frac in range(0, 256):  # 8-bit fractional
            clkdiv = div_int + div_frac / 256.0
            sm_freq = system_freq / clkdiv

            wrap_est = system_freq / (target_pwm_freq * instr_per_loop * clkdiv) - 1
            wrap = int(round(wrap_est))

            if not (1 <= wrap <= 0xFFFFFFFF):
                continue

            actual_freq = system_freq / ((wrap + 1) * instr_per_loop * clkdiv)
            freq_error = abs(actual_freq - target_pwm_freq)
            if freq_error / target_pwm_freq < tolerance:
                duty = int(round((wrap + 1) * duty_percent / 100.0))
                if verbose:
                    print("-" * 60)
                    print("[FAST PWM PIO] Perfect configuration found:")
                    print(f"  System Frequency     : {system_freq} Hz")
                    print(f"  Target PWM Frequency : {target_pwm_freq} Hz")
                    print(f"  Duty Cycle           : {duty_percent:.3f}%")
                    print(f"  Instructions/Loop    : {instr_per_loop}")
                    print("  Result:")
                    print(f"   - Duty Cycles = {duty}, Total Period Cycles = {wrap}")
                    print(f"   - Actual Frequency = {actual_freq:.9f} Hz")
                    print(f"   - div = {clkdiv:.9f} (int={div_int}, frac={div_frac})")
                    print(f"   - sm_freq = {sm_freq:.9f} Hz")

                return {
                    "sm_freq": sm_freq,
                    "duty": duty,
                    "wrap": wrap,
                    "div_int": div_int,
                    "div_frac": div_frac,
                    "actual_freq": actual_freq,
                    "exact": True,
                }

            # Save configuration with the smallest frequency difference
            if freq_error < min_error:
                min_error = freq_error
                best_config = {
                    "sm_freq": sm_freq,
                    "duty": int(round((wrap + 1) * duty_percent / 100.0)),
                    "wrap": wrap,
                    "div_int": div_int,
                    "div_frac": div_frac,
                    "actual_freq": actual_freq,
                    "exact": False,
                }

    if best_config is None:
        raise ValueError("No valid configuration found.")
    if verbose:
        print("-" * 60)
        print("[FAST PWM PIO] No perfect configuration. Using best available:")
        print(f"  System Frequency     : {system_freq} Hz")
        print(f"  Target PWM Frequency : {target_pwm_freq} Hz")
        print(f"  Duty Cycle           : {duty_percent:.3f}%")
        print(f"  Instructions/Loop    : {instr_per_loop}")
        print("  Result:")
        div = best_config["div_int"] + best_config["div_frac"] / 256.0
        print(
            f"   - Duty Cycles = {best_config['duty']}, Total Period Cycles = {best_config['wrap']}"
        )
        print(f"   - Actual Frequency = {best_config['actual_freq']:.9f} Hz")
        print(f"   - div = {div:.9f}")
        print(
            f"   - div_int = {best_config['div_int']}, div_frac = {best_config['div_frac']}"
        )
    return best_config


# This run the state machine at the system frequency and will try to approximate the high cycles and the total period cycles by simply division, which won't be exact, but obviously way faster.
def calculate_pwm_pio_parameters_simple(
    system_freq: float,
    target_pwm_freq: float,
    duty_percent: float,
    max_cycles: int = 2**32 - 1,
    instr_per_loop=3,
    verbose: bool = DEBUG,
):
    if not (0.0 <= duty_percent <= 100.0):
        raise ValueError("Duty cycle must be between 0 and 100")
    if target_pwm_freq <= 0 or system_freq <= 0:
        raise ValueError("Frequencies must be positive")

    period_cycles = int(system_freq / (target_pwm_freq * instr_per_loop))
    if period_cycles > max_cycles:
        raise ValueError("Requested period too long for PIO")
    duty_cycles = (duty_percent * period_cycles + 50) // 100
    actual_freq = system_freq / (period_cycles * instr_per_loop)

    if verbose:
        print("-" * 60)
        print("[SIMPLE PWM PIO]")
        print(f"  System Frequency     : {system_freq} Hz")
        print(f"  Target PWM Frequency : {target_pwm_freq} Hz")
        print(f"  Duty Cycle           : {duty_percent:.3f}%")
        print(f"  Instructions/Loop    : {instr_per_loop}")
        print("  Result:")
        print(f"   - Duty Cycles       : {duty_cycles}")
        print(f"   - Total Period Cycles : {period_cycles}")
        print(f"   - Actual PWM Frequency : {actual_freq:.9f} Hz")

    return {
        "sm_freq": system_freq,  # state machine runs at system frequency
        "duty": duty_cycles,  # duty cycles
        "wrap": period_cycles,  # total cycles
        "actual_freq": actual_freq,
    }


# Class to handle the pulse counting
class PulseCounter:
    def __init__(
        self,
        pulse_counter_pio_program: PIO,
        timing_pulse_pio_program: PIO,
        pwm_pio_program: PIO,
        pulse_counter_pio_sm_id: int,
        timing_pulse_pio_sm_id: int,
        pwm_sm_id: int,
        input_pulse_pin: int,
        timing_pulse_pin: int,
        timing_pulse_frequency: int,
        timing_pulse_ratio: int,
        sideset_pulse_pin: int,
        pio_freq: int,
    ):
        """
        Initialize the PulseCounter class.

        :param pulse_counter_pio_program: PIO program to count pulses
        :param timing_pulse_pio_program: PIO program to generate timing pulses
        :param pwm_pio_program: PIO program to generate the PWM signal

        :param pulse_counter_pio_sm_id: State machine ID for the pulse counter
        :param timing_pulse_pio_sm_id: State machine ID for the timing pulse
        :param pwm_sm_id: State machine ID for the PWM signal

        :param input_pulse_pin: Pin to measure the frequency of the input pulse
        :param timing_pulse_pin: Pin to generate the timing pulses
        :param timing_pulse_frequency: Frequency of the timing pulse
        :param timing_pulse_ratio: Ratio of the timing pulse to the gate time
        :param sideset_pulse_pin: Pin to control the gate time (sideset pin)
        :param pio_freq: Frequency of the PIO state machines

        :return: None
        """
        # sm id
        self.pulse_counter_pio_sm_id = pulse_counter_pio_sm_id
        self.timing_pulse_pio_sm_id = timing_pulse_pio_sm_id
        self.pwm_sm_id = pwm_sm_id
        StateMachine(self.pulse_counter_pio_sm_id).active(0)
        StateMachine(self.timing_pulse_pio_sm_id).active(0)
        StateMachine(self.pwm_sm_id).active(0)
        # input pin for the waveform to be measured
        self.pulse_pin = Pin(input_pulse_pin, Pin.IN)
        # reference timing pulse pin to gate the frequency measurement
        pio_pwm_config = compute_best_pwm_pio_parameters_fast(
            system_freq=freq(),
            target_pwm_freq=timing_pulse_frequency,
            duty_percent=50,
        )

        pio_pwm_config_1 = calculate_pwm_pio_parameters_simple(
            system_freq=freq(),
            target_pwm_freq=timing_pulse_frequency,
            duty_percent=50,
        )
        self.pio_pwm_sm_freq = int(pio_pwm_config_1["sm_freq"])
        self.pio_pwm_level = int(pio_pwm_config_1["duty"])
        self.pio_pwm_period = int(pio_pwm_config_1["wrap"] + 1)

        if DEBUG:
            print(
                f"State Machine Freq: {self.pio_pwm_sm_freq}, PWM Period: {self.pio_pwm_period}, Duty Level: {self.pio_pwm_level}"
            )
        self.timing_pin = Pin(timing_pulse_pin, Pin.OUT)
        self.timing_pulse_ratio = timing_pulse_ratio
        self.timing_pulse_frequency = timing_pulse_frequency
        self.timing_interval_ms = 1000 / timing_pulse_frequency * timing_pulse_ratio
        self.pio_pwm = StateMachine(
            pwm_sm_id,
            pwm_pio_program,
            freq=self.pio_pwm_sm_freq,
            in_base=self.timing_pin,
            jmp_pin=self.timing_pin,
            sideset_base=self.timing_pin,
        )

        # sideset pin to control the gate time
        self.sideset_pin = Pin(sideset_pulse_pin, Pin.OUT, value=0)
        # setup the pulse counter state machine
        self.pulse_counter_pio_sm = StateMachine(
            self.pulse_counter_pio_sm_id,
            pulse_counter_pio_program,
            freq=pio_freq,
            in_base=self.pulse_pin,
            jmp_pin=self.sideset_pin,
            sideset_base=self.sideset_pin,
        )
        # setup the timing pulse state machine
        self.timing_pulse_pio_sm = StateMachine(
            self.timing_pulse_pio_sm_id,
            timing_pulse_pio_program,
            freq=pio_freq,
            in_base=self.timing_pin,
            set_base=self.timing_pin,
            sideset_base=self.sideset_pin,
        )
        self.set_timing_pulse_ratio(self.timing_pulse_pio_sm, self.timing_pulse_ratio)
        self.pio_pwm_init(self.pio_pwm, self.pio_pwm_period, self.pio_pwm_level)

        if DEBUG:
            print(
                f"Pulse Counter SM ID: {self.pulse_counter_pio_sm_id}, Timing Pulse SM ID: {self.timing_pulse_pio_sm_id}, Timing Pulse Frequency: {timing_pulse_frequency} Hz, Timing Interval: {self.timing_interval_ms} ms"
            )

    def pio_pwm_init(self, sm: StateMachine, period: int, level: int):
        sm.active(0)
        sm.restart()
        sm.put(period)  # Load ISR (period) and OSR (level)
        sm.exec("pull(noblock)")
        sm.exec("mov(isr, osr)")
        sm.put(level)
        sm.exec("pull(noblock)")
        sm.exec("mov(x, osr)")
        sm.active(1)

    def set_timing_pulse_ratio(self, sm: StateMachine, timing_pulse_ratio: int):
        """
        On the fly set the timing pulse ratio, this will change the gate time.

        :param timing_pulse_ratio: Ratio of the timing pulse to the gate time
        :return: None
        """
        if timing_pulse_ratio <= 1:
            raise ValueError("Timing pulse ratio must be greater than 1")
        self.timing_pulse_ratio = timing_pulse_ratio
        sm.active(0)
        sm.put(self.timing_pulse_ratio - 1)
        sm.exec("pull(noblock)")
        sm.exec("mov(y, osr)")
        sm.exec("mov(x, y)")
        sm.restart()
        sm.active(1)

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
        self.pulse_counter_pio_sm.active(0)
        self.timing_pulse_pio_sm.active(0)
        self.pio_pwm.active(0)

        self.pulse_counter_pio_sm.restart()
        self.timing_pulse_pio_sm.restart()
        self.pio_pwm.restart()

        self.set_timing_pulse_ratio(self.timing_pulse_pio_sm, self.timing_pulse_ratio)
        self.pio_pwm_init(self.pio_pwm, self.pio_pwm_period, self.pio_pwm_level)
        self.start()

    def start(self):
        self.pulse_counter_pio_sm.active(1)
        self.timing_pulse_pio_sm.active(1)

    def stop(self):
        self.pulse_counter_pio_sm.active(0)
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
            pwm_pio_program=pwm_pio,
            pulse_counter_pio_sm_id=PULSE_COUNTER_SM_ID,
            timing_pulse_pio_sm_id=TIMING_PULSE_SM_ID,
            pwm_sm_id=PWM_SM_ID,
            input_pulse_pin=INPUT_PULSE_PIN_ABSOLUTE,
            timing_pulse_pin=TIMING_PULSE_PIN_ABSOLUTE,
            timing_pulse_frequency=TIMING_PULSE_FREQUENCY,
            timing_pulse_ratio=TIMING_PULSE_RATIO,
            sideset_pulse_pin=SIDESET_PIN_ABSOLUTE,
            pio_freq=freq(),
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
                while pulse_count == -1:
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
        print("Pulse counter stopped. Exiting...")


if __name__ == "__main__":
    main()
