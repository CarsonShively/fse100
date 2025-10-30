#!/usr/bin/env python3

import time, threading, subprocess, math, numbers
from concurrent.futures import ThreadPoolExecutor
# sudo apt install -y python3-rpi.gpio
import RPi.GPIO as GPIO
import os
from enum import IntEnum
from typing import Optional, Union
# sudo apt update
# sudo apt install -y espeak-ng

TEMP_THRESHOLD_C = 45.0 # test = 32 C, threshold = 45 C

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

TEMP_CLEAR_DELTA = 1.0 
TEMP_MIN_PERIOD  = 0.25
BUZZ_ON_S        = 0.20 
BUZZ_OFF_S       = 0.80

_state_lock = threading.Lock()
_overheat   = False
_state_evt  = threading.Event()

def set_overheat(new_state: bool):
    global _overheat
    with _state_lock:
        if new_state != _overheat:
            _overheat = new_state
            _state_evt.set()

def get_overheat():
    with _state_lock:
        return _overheat

def buzzer_worker(stop_evt: threading.Event):
    buzzing  = False
    on_phase = False
    next_t   = 0.0

    while not stop_evt.is_set():
        _state_evt.wait(timeout=0.05)
        if stop_evt.is_set():
            break
        hot = get_overheat()
        _state_evt.clear()

        now = time.monotonic()

        if hot:
            if not buzzing:
                buzzing  = True
                on_phase = True
                on()
                next_t   = now + BUZZ_ON_S

            if buzzing and now >= next_t:
                if on_phase:
                    off()
                    on_phase = False
                    next_t   = now + BUZZ_OFF_S
                else:
                    on()
                    on_phase = True
                    next_t   = now + BUZZ_ON_S
        else:
            if buzzing:
                off()
                buzzing  = False
                on_phase = False
                next_t   = 0.0

def handle_temp(stop_evt: threading.Event):
    last = 0.0
    hot  = False
    trip = TEMP_THRESHOLD_C
    clr  = TEMP_THRESHOLD_C - TEMP_CLEAR_DELTA

    while not stop_evt.is_set():
        now = time.monotonic()
        remain = TEMP_MIN_PERIOD - (now - last)
        if remain > 0 and stop_evt.wait(remain):
            break
        last = time.monotonic()

        try:
            t = temp()
        except Exception:
            t = None

        if not (isinstance(t, numbers.Real) and math.isfinite(t)):
            continue

        new_hot = (t > trip) if not hot else (t >= clr)
        if new_hot != hot:
            hot = new_hot
            set_overheat(hot)

MAX_DISTANCE_CM = 40.0
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

_slot_lock = threading.Lock()
_slot_seq = 0
_slot_msg = None
_slot_bucket: int | None = None
_slot_evt = threading.Event()

MIN_PERIOD = 0.06
DEBOUNCE_S = 0.15
cand_bucket = None
cand_since = 0.0

def say_overwrite(msg: str, bucket: Optional[Union["DistBucket", int]] = None):
    global _slot_seq, _slot_msg, _slot_bucket
    with _slot_lock:
        _slot_seq += 1
        _slot_msg = str(msg)
        _slot_bucket = int(bucket) if bucket is not None else None
        _slot_evt.set()

say = say_overwrite

def speaker(stop_evt: threading.Event):
    last_seen_seq = -1
    while not stop_evt.is_set():
        _slot_evt.wait(timeout=0.1)
        if stop_evt.is_set():
            break
        
        if not _slot_evt.is_set():
            continue

        
        while True:
            with _slot_lock:
                seq = _slot_seq
                msg = _slot_msg
                bucket = _slot_bucket
                _slot_evt.clear()
            time.sleep(0.003)
            with _slot_lock:
                if seq == _slot_seq:
                    break

        if msg is None or seq == last_seen_seq:
            continue

        try:
            subprocess.run(["espeak-ng", "-s180", "-a200", "-ven", str(msg)], check=True)
        except Exception:
            pass

        last_seen_seq = seq

class DistBucket(IntEnum):
    START        = -1
    INVALID      = 0
    CM_0_2       = 1
    CM_2_6       = 2
    CM_6_10      = 3
    CM_10_15     = 4
    CM_15_20     = 5
    CM_20_25     = 6
    CM_25_30     = 7
    CM_30_35     = 8
    CM_35_MAX    = 9
    OUT_OF_RANGE = 10

def buckets_only_map(dist: Optional[float]) -> DistBucket:
    if dist is None:
        return DistBucket.INVALID
    if dist > MAX_DISTANCE_CM:
        return DistBucket.OUT_OF_RANGE
    if dist <= 2:   return DistBucket.CM_0_2
    if dist <= 6:   return DistBucket.CM_2_6
    if dist <= 10:  return DistBucket.CM_6_10
    if dist <= 15:  return DistBucket.CM_10_15
    if dist <= 20:  return DistBucket.CM_15_20
    if dist <= 25:  return DistBucket.CM_20_25
    if dist <= 30:  return DistBucket.CM_25_30
    if dist <= 35:  return DistBucket.CM_30_35
    return DistBucket.CM_35_MAX

def handle_distance(stop_evt: threading.Event):
    global cand_bucket, cand_since
    bucket: DistBucket = DistBucket.START
    last = 0.0

    while not stop_evt.is_set():
        now = time.monotonic()
        remain = MIN_PERIOD - (now - last)
        if remain > 0 and stop_evt.wait(remain):
            break
        last = time.monotonic()

        try:
            dist_cm = distance()  
        except Exception:
            dist_cm = None
        if not (isinstance(dist_cm, numbers.Real) and math.isfinite(dist_cm) and dist_cm >= 0.0):
            dist_cm = None

        new_bucket = buckets_only_map(dist_cm)

        if new_bucket != bucket:
            t = time.monotonic()
            if new_bucket != cand_bucket:
                cand_bucket = new_bucket
                cand_since = t
            elif t - cand_since >= DEBOUNCE_S:
                if new_bucket == DistBucket.INVALID:
                    say_overwrite("invalid read", new_bucket)
                elif new_bucket == DistBucket.OUT_OF_RANGE:
                    say_overwrite("out of range", new_bucket)
                else:
                    say_overwrite(f"{(dist_cm or 0):.0f} centimeters", new_bucket)
                bucket = new_bucket
                cand_bucket = None
        else:
            cand_bucket = None

def main():
    stop_evt = threading.Event()
    try:
        GPIO.setmode(GPIO.BOARD)
        temp_setup()
        dist_setup()
        buzz_setup(Buzzer)
        
        print("\nactive...")

        with ThreadPoolExecutor(max_workers=4) as executor:
            futures = [
                executor.submit(handle_temp, stop_evt),
                executor.submit(handle_distance, stop_evt),
                executor.submit(speaker, stop_evt),
                executor.submit(buzzer_worker, stop_evt),
            ]
            try:
                while True:
                    time.sleep(1.0)
            except KeyboardInterrupt:
                print("\nshut down...")
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

#temp:
#   state true == buzzer on
#   state false == buzzer off
#   hyst == error margin (not delay)

#dist:
#   send debounced value (confirmed bucket changes) to overwrite slot
#   dedupe replaces slot with most recent debounced value
#   async speaker reads from slot immediately and idles if slot is empty 
