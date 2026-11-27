import argparse 
import math
import re #to identify correct GPIO chip by line name
import time
import socket
import json

PULSE_GPIO = 18 #Pi pin 12/ On Driver PU+ 
DIR_GPIO = 23 #Pi pin 16/ On Driver DR+

PULSES_PER_REV = 1600 # adjusted by switches on stepper driver

center_position = 90.0 #paddle set at 90 degrees
min_position = 0.0
max_position = 180.0
START_HZ = 180.0 #pulses per second, (HZ x 60)/PULSES_PER_REV, 6.75RPM
ACCEL_HZ_PER_SEC = 12000.0 #acceleration of pulse frequency per second
PULSES_PER_BATCH = 12 #sends 12 pulses at current frequency
max_rpm = 150.0

HOST = "0.0.0.0"
PORT = 5005
MAX_PACKET_AGE_S = 0.5

def start_server(port=PORT):
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        server.bind((HOST, port))
    except BaseException:
        server.close()
        raise
    return server

#takes an angle and calculates the number of pulses to get to that angle
#a negative will make it turn towards 0 degrees
def target_steps(angle):
    return round((angle - center_position) * PULSES_PER_REV/ 360.0)

#pulses are sent in batches, accelerates, reaches max speed, decelerates
#total_pulses: pulses required to reach target, peak_hz: max pulse frequency allowed
def plan_segments(total_pulses, peak_hz):
    sent = 0
    while sent < total_pulses:
        num_of_pulses = min(PULSES_PER_BATCH, total_pulses - sent)
        
        #compare distances from batch midpoint to START or END and take the smallest one
        nearest_distance = min(sent + num_of_pulses/2.0, total_pulses - sent - num_of_pulses/2.0)
        # (velocity) v^2 = vi^2 + 2*acc*dist 
        rate = math.sqrt(START_HZ**2 + 2 * ACCEL_HZ_PER_SEC * max(nearest_distance, 0))

        yield num_of_pulses, min(peak_hz, rate)
        sent = sent + num_of_pulses

#find gpio controller that owns Pi header pins
#requested: provided controller number
def open_rp1_chip(lgpio, requested):
    if requested is not None:
        chips = [requested]
    else: #if controller num not identified it will loop 0-9
        chips = range(0, 10)

    for chip in chips:
        try: #open to see GPIO line names
            handle = lgpio.gpiochip_open(chip)
        except Exception:
            continue
        
        try: #read GPIO line names
            info_a = lgpio.gpio_get_line_info(handle, PULSE_GPIO)
            info_b = lgpio.gpio_get_line_info(handle, DIR_GPIO)

            names_a, names_b = [], []
            for x in info_a:
                names_a.append(str(x))
            for x in info_b:
                names_b.append(str(x))
            
            #check if string matches for both pins
            a_ok = False
            for s in names_a:
                if s.upper() == "GPIO18":
                    a_ok = True
                    break
            b_ok = False
            for s in names_b:
                if s.upper() == "GPIO23":
                    b_ok = True
                    break
            
            if a_ok and b_ok:
                return handle, chip
        except Exception:
            pass
        lgpio.gpiochip_close(handle)
    raise RuntimeError('Cant identify Raspberry Pi GPIO header')

class Goalie:
    def __init__(self, peak_rpm, chip=None, dry_run=False):
        self.dry_run = dry_run
        self.peak_hz = peak_rpm * PULSES_PER_REV/60.0
        self.current_steps = 0 #number of steps away from 90 degrees
        self.handle = None #refrence for GPIO controller
        self.lgpio = None
        self.position_known = dry_run

        if not dry_run:
            import lgpio
            self.lgpio = lgpio
            self.handle, chip_number = open_rp1_chip(lgpio, chip)
            try: #configure pins as output pins, set initially to low (0)
                lgpio.gpio_claim_output(self.handle, PULSE_GPIO, 0)
                lgpio.gpio_claim_output(self.handle, DIR_GPIO, 0)
            except BaseException:
                lgpio.gpiochip_close(self.handle)
                raise
            print(f'using GPIO chip: {chip_number}; BCM pins: 18(pulse) and 23(direction)')

    def stop(self): #stop motor pulses
        if self.handle is not None:
            try:
                #(handle, gpio, pusle frequency, duty cycle)
                self.lgpio.tx_pwm(self.handle, PULSE_GPIO, 0, 0) #cancels pulse generation
            except Exception as err:
                print(f'WARNING: PWM cancellation failed: {err}')
            try:
                self.lgpio.gpio_write(self.handle, PULSE_GPIO, 0)
            except Exception as err:
                print(f'WARNING: Could not set pulse output low: {err}')

    def close(self): #close GPIO resources when program exits
        if self.handle is not None:
            self.stop()
            self.lgpio.gpiochip_close(self.handle)
            self.handle = None

    def move_to(self, angle): #move to predicted angle
        if not math.isfinite(angle) or not min_position <= angle <= max_position:
            raise ValueError('Angle must be a number from 0 to 180 degrees')
        
        if not self.position_known:
            raise RuntimeError('paddle may not be centered. Recenter physically before restarting')

        wanted_steps = target_steps(angle) #steps needed from 90 -> target angle
        delta = wanted_steps - self.current_steps #steps needed to move from current position -> target
        total = abs(delta) #total steps needed to move

        if not total: #if total is 0, paddle is at requested position
            print(f'already at commanded {angle:g} degrees')
            return True

        #set direction
        if delta > 0: #move toward 180 (clockwise)
            direction = 0
        else: #move toward 0 (counter clockwise)
            direction = 1

        #build the planned batches and their acceleration rates
        batches = list(plan_segments(total, self.peak_hz))
        expected_time = 0 #calculate total time to execute all batches
        for num_of_pulses, frequency in batches:
            batch_time = num_of_pulses / frequency
            expected_time = expected_time + batch_time
        print(f'moving {total} pulses to {angle:g} degrees '
              f'(ideal pulse time ~{expected_time:.3f} seconds)')

        if self.dry_run: #simulate
            self.current_steps = wanted_steps
            print('dry run: no gpio signals generated')
            return True
        
        try: #to protect movement so a failure invalidates position tracking
            self.lgpio.gpio_write(self.handle, DIR_GPIO, direction) #set direction
            time.sleep(0.003)

            # Send one batch at a time and wait for it to finish.
            for num_of_pulses, frequency in batches:
                self.lgpio.tx_pwm(self.handle, PULSE_GPIO, frequency, 50.0, 0, num_of_pulses)
                deadline = time.monotonic() + max(1.0, num_of_pulses / frequency * 4)
                while self.lgpio.tx_busy(self.handle, PULSE_GPIO, 0):
                    if time.monotonic() > deadline:
                        raise TimeoutError("PWM batch did not finish on time")
                    time.sleep(0.001)

            #update current position
            self.current_steps = wanted_steps
            print(f'command complete; assumed position: {angle:g} degrees')
            return True
        except BaseException:
            self.position_known = False
            self.stop()
            raise

def main():
    parser = argparse.ArgumentParser(description='Goalie UDP motor test (manual centering; no limit switches)')
    parser.add_argument('--port', type=int, default=PORT)
    parser.add_argument('--max-rpm', type=float, default=10,
                        help='Initial speed ceiling (default 10 RPM; maximum 150).')
    parser.add_argument('--chip', type=int, default=None,
                        help='GPIO chip number; autodetect if omitted.')
    parser.add_argument('--dry-run', action='store_true',
                        help='Receive and simulate angles without opening GPIO or moving the motor.')
    args = parser.parse_args()
    if not math.isfinite(args.max_rpm) or not 10 <= args.max_rpm <= max_rpm:
        parser.error('Choose --max-rpm between 10 and 150 for initial testing.')
    print('GOALIE | 0-180 degrees | center = 90 degrees')
    if args.dry_run:
        print("Dry run: GPIO disabled; no physical motor movement.")
    else:
        print("WARNING: No homing or physical limit switches. Keep an emergency power cutoff accessible.")
        print("With motor power OFF, manually center the paddle at 90 degrees.")
        print("Clear the paddle's travel area, then power ON the motor.")
        if input("Type CENTERED to arm: ").strip() != "CENTERED":
            print("Not armed. Exiting.")
            return
    motor = Goalie(args.max_rpm, args.chip, dry_run=args.dry_run)
    motor.position_known = True  # Manual centering confirmed, or simulated center.
    try:

        server = start_server(args.port)
        print(f"Listening for UDP angle commands on port {args.port}...")
        last_seq = -1

        while True:
            data, addr = server.recvfrom(1024)
            try:
                msg = json.loads(data.decode())
                seq = int(msg["seq"])
                sent_at = float(msg["ts"])
                angle = float(msg["angle"])
            except (KeyError, ValueError, TypeError, OverflowError):
                print(f"Dropping malformed packet from {addr}")
                continue

            if seq <= last_seq:
                print(f"Dropping out-of-order packet (seq={seq}, last={last_seq})")
                continue
            if not math.isfinite(sent_at) or not math.isfinite(angle):
                print(f"Dropping non-finite packet from {addr}")
                continue

            age = time.time() - sent_at
            if age > MAX_PACKET_AGE_S or age < -MAX_PACKET_AGE_S:
                print(f"Dropping packet (seq={seq}, age={age:.3f}s; stale or clock mismatch)")
                continue
            if not min_position <= angle <= max_position:
                print(f"Dropping out-of-range angle {angle} from {addr}")
                continue

            # Keep initial physical tests near center without limit switches.
            if not args.dry_run and not 85.0 <= angle <= 95.0:
                print(f"Dropping angle {angle:g}: physical test range is 85-95 degrees")
                continue

            last_seq = seq
            print(f"seq={seq} angle={angle:.1f} age={age * 1000:.0f}ms from {addr}")
            # Move to the received target (or simulate in dry-run mode).
            motor.move_to(angle)
    except (KeyboardInterrupt, EOFError):
        print('\nInterrupted. Recenter manually before restarting.')
    finally:
        motor.close()
        if 'server' in locals():
            server.close()
        print("UDP connection test closed")


if __name__ == '__main__':
    main()
