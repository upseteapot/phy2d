#!/bin/bash

set -x
rm -f test
gcc -Wall -Wextra -pedantic test.c phys2d.c -o test -lraylib -lm -I./
./test

