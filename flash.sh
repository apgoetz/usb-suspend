#!/bin/bash
if (( $# >= 1)); then
    cargo objcopy --bin $1 "$2" -- -O ihex  program.hex && \
	st-flash --format ihex write program.hex && \
	rm program.hex	
else
        echo "Usage:"
        echo "$0 [--release] <filename of firmware in ELF format>"
        exit 1
fi


