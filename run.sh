#!/bin/bash

set -x
rm -f test
gcc -Wall -Wextra -pedantic test.c phy2d.c -o test -lraylib -lm -I./
./test

