#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import pigpio

if __name__ == '__main__':
    pi = pigpio.pi()
    pi.set_mode(18, pigpio.OUTPUT)
    pi.set_PWM_frequency(18, 50)
    pi.set_PWM_range(18, 10000)
    pi.set_PWM_dutycycle(18, 500)
