#!/bin/sh

openocd -f openocd.cfg -c init -c reset
make program
