#!/usr/bin/env python3

import time, threading, subprocess, math, numbers
from concurrent.futures import ThreadPoolExecutor
# sudo apt install -y python3-rpi.gpio
import RPi.GPIO as GPIO
import os

TEMP_THRESHOLD_C = 45.0

ds18b20 = ''
# pi setupt == exactly as in website (one-to-one)
def temp_setup():
    global ds18b20
    for i in os.listdir('/sys/bus/w1/devices'):
        if i != 'w1_bus_master1':
            ds18b20 = '28-031590bf4aff' # sensor address

def temp():
    location = '/sys/bus/w1/devices/' + ds18b20 + '/w1_slave'
    tfile = open(location)
    text = tfile.read()
    tfile.close()
    secondline = text.split("\n")[1]
    temperaturedata = secondline.split(" ")[9]
    temperature = float(temperaturedata[2:])
    temperature = temperature / 1000
    return temperature

Buzzer = 13

def buzz_setup(pin):
    global BuzzerPin
    BuzzerPin = pin
    GPIO.setup(BuzzerPin, GPIO.OUT, initial=GPIO.HIGH)

def on():
    GPIO.output(BuzzerPin, GPIO.LOW)

def off():
    GPIO.output(BuzzerPin, GPIO.HIGH)

def beep(x):
    on()
    time.sleep(x)
    off()
    time.sleep(x)

def handle_temp(stop_evt):
    while not stop_evt.is_set():
        try:
            temp_c = temp()
        except Exception:
            temp_c = None
        if temp_c is not None and temp_c > TEMP_THRESHOLD_C:
            beep(0.2)
        if stop_evt.wait(0.1):
            break

MAX_DISTANCE_CM = 60.0
TRIG = 11
ECHO = 12

def dist_setup():
    GPIO.setup(TRIG, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ECHO, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    time.sleep(0.05)
    
def distance(timeout_s=0.02):
    GPIO.output(TRIG, GPIO.LOW); time.sleep(2e-6)
    GPIO.output(TRIG, GPIO.HIGH); time.sleep(10e-6)
    GPIO.output(TRIG, GPIO.LOW)

    t0 = time.perf_counter()
    while GPIO.input(ECHO) == 0:
        if time.perf_counter() - t0 > timeout_s:
            return None
    start = time.perf_counter()

    while GPIO.input(ECHO) == 1:
        if time.perf_counter() - start > timeout_s:
            return None
    end = time.perf_counter()

    return (end - start) * 343.0 / 2.0 * 100.0 

# sudo apt update
# sudo apt install -y espeak-ng
def say(text, rate=160, volume=200, voice="en"):
    try:
        subprocess.run(
            ["espeak-ng", f"-s{rate}", f"-a{volume}", f"-v{voice}", str(text)],
            check=True
        )
    except Exception:
        pass

def buckets(dist, bucket):
    if dist is None:
        return bucket

    match True:
        case _ if dist <= 10:
            if bucket != 1:
                bucket = 1
                say(f"{dist:.0f}")
        case _ if 10 < dist <= 20:
            if bucket != 2:
                bucket = 2
                say(f"{dist:.0f}")
        case _ if 20 < dist <= 30:
            if bucket != 3:
                bucket = 3
                say(f"{dist:.0f}")
        case _ if 30 < dist <= 40:
            if bucket != 4:
                bucket = 4
                say(f"{dist:.0f}")
        case _ if 40 < dist <= 50:
            if bucket != 5:
                bucket = 5
                say(f"{dist:.0f}")
        case _ if 50 < dist <= MAX_DISTANCE_CM:
            if bucket != 6:
                bucket = 6
                say(f"{dist:.0f}")
        case _:
            pass

    return bucket

def handle_distance(stop_evt):
    bucket = -1
    while not stop_evt.is_set():
        try:
            dist_cm = distance()
        except Exception:
            dist_cm = None
        if isinstance(dist_cm, numbers.Real) and math.isfinite(dist_cm) and 0.0 <= dist_cm <= MAX_DISTANCE_CM:
            pass
        else:
            dist_cm = None
        bucket = buckets(dist_cm, bucket)
        if stop_evt.wait(1.0):
            break

def main():
    stop_evt = threading.Event()
    try:
        GPIO.setmode(GPIO.BOARD)
        temp_setup()
        dist_setup()
        buzz_setup(Buzzer)

        with ThreadPoolExecutor(max_workers=2) as executor:
            futures = [
                executor.submit(handle_temp, stop_evt),
                executor.submit(handle_distance, stop_evt),
            ]
            try:
                while True:
                    time.sleep(1.0)
            except KeyboardInterrupt:
                print("\n[CTRL-C] shutting down…")
            finally:
                stop_evt.set()
                for f in futures:
                    try:
                        f.result(timeout=2.0)
                    except Exception:
                        pass

    except Exception as e:
        print(f"Error: {e}")
    finally:
        try:
            off()
        except Exception:
            pass
        try:
            GPIO.cleanup()
        except Exception as e:
            print(f"GPIO cleanup failed: {e}")

if __name__ == "__main__":
    main()
