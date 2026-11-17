#!/bin/bash

mc_reg_read() {
    local name=$1
    local rge_addr=$2
    local reg_size=$3
    local spi_dev=""
    local value=0x60000000

    if [[ "$name" == "lan9373" ]]; then
        echo "LAN9373  reg addr: $rge_addr,  reg size: $reg_size"
	spi_dev="/dev/spidev0.1"
    elif [[ "$name" == "ksz9567" ]]; then
        echo "KSZ9567  reg addr: $rge_addr,  reg size: $reg_size"
        spi_dev="/dev/spidev0.0"
    else
        echo "Unknown device: $name"
	exit 1
    fi

    local result=$(( (rge_addr << 13) + value ))
    local hex_result=$(printf "%X" "$result")
    
    local formatted_result=""
    for (( i=0; i<${#hex_result}; i+=2 )); do
        formatted_result+="\\x${hex_result:$i:2}"
    done

    for (( i=0; i<$reg_size; i++ )); do
        formatted_result+="\\x00"
    done

    spidev_test -HOv -D $spi_dev -p $formatted_result
}


# function: mc_reg_read
# $1: "lan9373" or "ksz9567"
# $2: Register address
# $3: Register address length (in bytes)
# Example: Read the 0x1 register of ksz9567, which is 1 byte in size 
mc_reg_read "ksz9567" 0x1 1 
mc_reg_read "KSZ9567R" 0x0 1 
mc_reg_read "KSZ9567R" 0x1 1 
mc_reg_read "KSZ9567R" 0x2 1 
mc_reg_read "KSZ9567R" 0x3 1 
mc_reg_read "KSZ9567R" 0x6 1 
mc_reg_read "KSZ9567R" 0x10 4 
mc_reg_read "KSZ9567R" 0x14 4 
mc_reg_read "KSZ9567R" 0x18 4 
mc_reg_read "KSZ9567R" 0x1C 4 
mc_reg_read "KSZ9567R" 0x100 1 
mc_reg_read "KSZ9567R" 0x103 1
 mc_reg_read "KSZ9567R" 0x104 4 
mc_reg_read "KSZ9567R" 0x10D 1 
mc_reg_read "KSZ9567R" 0x110 4 
mc_reg_read "KSZ9567R" 0x120 4 
mc_reg_read "KSZ9567R" 0x124 4 
mc_reg_read "KSZ9567R" 0x128 4 
mc_reg_read "KSZ9567R" 0x201 1 
mc_reg_read "KSZ9567R" 0x210 4 
mc_reg_read "KSZ9567R" 0x300 1 
mc_reg_read "KSZ9567R" 0x302 1 
mc_reg_read "KSZ9567R" 0x303 1 
mc_reg_read "KSZ9567R" 0x304 1 
mc_reg_read "KSZ9567R" 0x305 1 
mc_reg_read "KSZ9567R" 0x306 1 
mc_reg_read "KSZ9567R" 0x307 1 
mc_reg_read "KSZ9567R" 0x308 2 
mc_reg_read "KSZ9567R" 0x30A 2
 mc_reg_read "KSZ9567R" 0x30E 2 
mc_reg_read "KSZ9567R" 0x310 1 
mc_reg_read "KSZ9567R" 0x311 1 
mc_reg_read "KSZ9567R" 0x312 1 
mc_reg_read "KSZ9567R" 0x313 1
 mc_reg_read "KSZ9567R" 0x314 1 
mc_reg_read "KSZ9567R" 0x315 1 
mc_reg_read "KSZ9567R" 0x316 2 
mc_reg_read "KSZ9567R" 0x318 2 
mc_reg_read "KSZ9567R" 0x31A 2 
mc_reg_read "KSZ9567R" 0x320 4 
mc_reg_read "KSZ9567R" 0x324 4 
mc_reg_read "KSZ9567R" 0x328 4 
mc_reg_read "KSZ9567R" 0x330 1 
mc_reg_read "KSZ9567R" 0x331 1 
mc_reg_read "KSZ9567R" 0x332 1 
mc_reg_read "KSZ9567R" 0x333 1 
mc_reg_read "KSZ9567R" 0x334 1 
mc_reg_read "KSZ9567R" 0x335 1 
mc_reg_read "KSZ9567R" 0x336 1 
mc_reg_read "KSZ9567R" 0x338 1 
mc_reg_read "KSZ9567R" 0x339 1 
mc_reg_read "KSZ9567R" 0x33A 1 
mc_reg_read "KSZ9567R" 0x33B 1 
mc_reg_read "KSZ9567R" 0x33E 1 
mc_reg_read "KSZ9567R" 0x340 1 
mc_reg_read "KSZ9567R" 0x341 1 
mc_reg_read "KSZ9567R" 0x342 1 
mc_reg_read "KSZ9567R" 0x343 1 
mc_reg_read "KSZ9567R" 0x344 1 
mc_reg_read "KSZ9567R" 0x345 1 
mc_reg_read "KSZ9567R" 0x346 1 
mc_reg_read "KSZ9567R" 0x347 1 
mc_reg_read "KSZ9567R" 0x348 1 
mc_reg_read "KSZ9567R" 0x349 1 
mc_reg_read "KSZ9567R" 0x34A 1 
mc_reg_read "KSZ9567R" 0x34B 1 
mc_reg_read "KSZ9567R" 0x34C 1 
mc_reg_read "KSZ9567R" 0x34D 1 
mc_reg_read "KSZ9567R" 0x34E 1 
mc_reg_read "KSZ9567R" 0x34F 1
 mc_reg_read "KSZ9567R" 0x350 1 
mc_reg_read "KSZ9567R" 0x351 1 
mc_reg_read "KSZ9567R" 0x352 1 
mc_reg_read "KSZ9567R" 0x353 1 
mc_reg_read "KSZ9567R" 0x354 1 
mc_reg_read "KSZ9567R" 0x355 1 
mc_reg_read "KSZ9567R" 0x350 1 
mc_reg_read "KSZ9567R" 0x357 1
 mc_reg_read "KSZ9567R" 0x358 1 
mc_reg_read "KSZ9567R" 0x359 1 
mc_reg_read "KSZ9567R" 0x35A 1 
mc_reg_read "KSZ9567R" 0x35B 1 
mc_reg_read "KSZ9567R" 0x35C 1
 mc_reg_read "KSZ9567R" 0x35D 1 
mc_reg_read "KSZ9567R" 0x35E 1
 mc_reg_read "KSZ9567R" 0x35F 1
 mc_reg_read "KSZ9567R" 0x370 1 
mc_reg_read "KSZ9567R" 0x378 1 
mc_reg_read "KSZ9567R" 0x37C 1 
mc_reg_read "KSZ9567R" 0x37D 1 
mc_reg_read "KSZ9567R" 0x390 4 
mc_reg_read "KSZ9567R" 0x400 4 
mc_reg_read "KSZ9567R" 0x404 4 
mc_reg_read "KSZ9567R" 0x408 4 
mc_reg_read "KSZ9567R" 0x40C 2 
mc_reg_read "KSZ9567R" 0x40E 1 
mc_reg_read "KSZ9567R" 0x410 4
 mc_reg_read "KSZ9567R" 0x414 4 
mc_reg_read "KSZ9567R" 0x418 4 
mc_reg_read "KSZ9567R" 0x41C 4 
mc_reg_read "KSZ9567R" 0x420 4 
mc_reg_read "KSZ9567R" 0x424 4 
mc_reg_read "KSZ9567R" 0x428 4 
mc_reg_read "KSZ9567R" 0x42C 4
 mc_reg_read "KSZ9567R" 0x500 2 
mc_reg_read "KSZ9567R" 0x502 2 
mc_reg_read "KSZ9567R" 0x504 2 
mc_reg_read "KSZ9567R" 0x506 2
 mc_reg_read "KSZ9567R" 0x508 2 
mc_reg_read "KSZ9567R" 0x50A 2 
mc_reg_read "KSZ9567R" 0x50C 2 
mc_reg_read "KSZ9567R" 0x50E 2
 mc_reg_read "KSZ9567R" 0x510 2 
mc_reg_read "KSZ9567R" 0x512 2 
mc_reg_read "KSZ9567R" 0x514 2
 mc_reg_read "KSZ9567R" 0x516 2 
mc_reg_read "KSZ9567R" 0x518 2 
mc_reg_read "KSZ9567R" 0x520 4 
mc_reg_read "KSZ9567R" 0x524 4 
mc_reg_read "KSZ9567R" 0x528 4
 mc_reg_read "KSZ9567R" 0x52C 4
 mc_reg_read "KSZ9567R" 0x530 4 
mc_reg_read "KSZ9567R" 0x534 4 
mc_reg_read "KSZ9567R" 0x538 4 
mc_reg_read "KSZ9567R" 0x53C 4 
mc_reg_read "KSZ9567R" 0x540 4 
mc_reg_read "KSZ9567R" 0x544 4 
mc_reg_read "KSZ9567R" 0x548 4 
mc_reg_read "KSZ9567R" 0x550 4 
mc_reg_read "KSZ9567R" 0x554 4 
mc_reg_read "KSZ9567R" 0x558 4 
mc_reg_read "KSZ9567R" 0x55C 4
 mc_reg_read "KSZ9567R" 0x560 4 
mc_reg_read "KSZ9567R" 0x564 4
mc_reg_read "KSZ9567R" 0x568 4 
mc_reg_read "KSZ9567R" 0x56C 4 
mc_reg_read "KSZ9567R" 0x570 4 
mc_reg_read "KSZ9567R" 0x574 4 
mc_reg_read "KSZ9567R" 0x578 4 
mc_reg_read "KSZ9567R" 0x57C 4 
mc_reg_read "KSZ9567R" 0x580 4 
mc_reg_read "KSZ9567R" 0x584 4 
mc_reg_read "KSZ9567R" 0x588 4 
mc_reg_read "KSZ9567R" 0x58C 4 
mc_reg_read "KSZ9567R" 0x590 4 
mc_reg_read "KSZ9567R" 0x594 4 
mc_reg_read "KSZ9567R" 0x598 4 
mc_reg_read "KSZ9567R" 0x59C 4 
mc_reg_read "KSZ9567R" 0x5A0 4 
mc_reg_read "KSZ9567R" 0x5A4 4 
mc_reg_read "KSZ9567R" 0x5A8 4 
mc_reg_read "KSZ9567R" 0x5AC 4 
mc_reg_read "KSZ9567R" 0x5B0 4
 mc_reg_read "KSZ9567R" 0xN000 1 
mc_reg_read "KSZ9567R" 0xN001 1 
mc_reg_read "KSZ9567R" 0xN013 1 
mc_reg_read "KSZ9567R" 0xN017 1 
mc_reg_read "KSZ9567R" 0xN01B 1 
mc_reg_read "KSZ9567R" 0xN01F 1
 mc_reg_read "KSZ9567R" 0xN020 1 
mc_reg_read "KSZ9567R" 0xN030 1 
mc_reg_read "KSZ9567R" 0xN100 2 
mc_reg_read "KSZ9567R" 0xN102 2 
mc_reg_read "KSZ9567R" 0xN104 2 
mc_reg_read "KSZ9567R" 0xN106 2 
mc_reg_read "KSZ9567R" 0xN108 2 
mc_reg_read "KSZ9567R" 0xN10A 2
 mc_reg_read "KSZ9567R" 0xN10C 2 
mc_reg_read "KSZ9567R" 0xN10E 2 
mc_reg_read "KSZ9567R" 0xN110 2 
mc_reg_read "KSZ9567R" 0xN112 2 
mc_reg_read "KSZ9567R" 0xN114 2 
mc_reg_read "KSZ9567R" 0xN11A 2
 mc_reg_read "KSZ9567R" 0xN11C 2 
mc_reg_read "KSZ9567R" 0xN11E 2 
mc_reg_read "KSZ9567R" 0xN112 2 
mc_reg_read "KSZ9567R" 0xN124 2 
mc_reg_read "KSZ9567R" 0xN126 2 
mc_reg_read "KSZ9567R" 0xN12A 2 
mc_reg_read "KSZ9567R" 0xN136 2 
mc_reg_read "KSZ9567R" 0xN138 2 
mc_reg_read "KSZ9567R" 0xN13E 2 
mc_reg_read "KSZ9567R" 0xN300 1 
mc_reg_read "KSZ9567R" 0xN301 1 
mc_reg_read "KSZ9567R" 0xN400 1 
mc_reg_read "KSZ9567R" 0xN401 1 
mc_reg_read "KSZ9567R" 0xN403 1
 mc_reg_read "KSZ9567R" 0xN410 1
 mc_reg_read "KSZ9567R" 0xN411 1
 mc_reg_read "KSZ9567R" 0xN412 1
 mc_reg_read "KSZ9567R" 0xN413 1 
mc_reg_read "KSZ9567R" 0xN414 1 
mc_reg_read "KSZ9567R" 0xN415 1 
mc_reg_read "KSZ9567R" 0xN416 1 
mc_reg_read "KSZ9567R" 0xN417 1 
mc_reg_read "KSZ9567R" 0xN420 1 
mc_reg_read "KSZ9567R" 0xN421 1 
mc_reg_read "KSZ9567R" 0xN422 1 
mc_reg_read "KSZ9567R" 0xN423 1
 mc_reg_read "KSZ9567R" 0xN500 4 
mc_reg_read "KSZ9567R" 0xN504 4 
mc_reg_read "KSZ9567R" 0xN600 1 
mc_reg_read "KSZ9567R" 0xN601 1 
mc_reg_read "KSZ9567R" 0xN602 1 
mc_reg_read "KSZ9567R" 0xN603 1
 mc_reg_read "KSZ9567R" 0xN604 1
 mc_reg_read "KSZ9567R" 0xN605 1 
mc_reg_read "KSZ9567R" 0xN606 1
 mc_reg_read "KSZ9567R" 0xN607 1 
mc_reg_read "KSZ9567R" 0xN608 1
 mc_reg_read "KSZ9567R" 0xN609 1 
mc_reg_read "KSZ9567R" 0xN60A 1 
mc_reg_read "KSZ9567R" 0xN60B 1 
mc_reg_read "KSZ9567R" 0xN60C 1 
mc_reg_read "KSZ9567R" 0xN60D 1 
mc_reg_read "KSZ9567R" 0xN60E 1 
mc_reg_read "KSZ9567R" 0xN60F 
1 mc_reg_read "KSZ9567R" 0xN610 1 
mc_reg_read "KSZ9567R" 0xN611 1 
mc_reg_read "KSZ9567R" 0xN612 1 
mc_reg_read "KSZ9567R" 0xN800 1 
mc_reg_read "KSZ9567R" 0xN801 1 
mc_reg_read "KSZ9567R" 0xN802 1 
mc_reg_read "KSZ9567R" 0xN803 1
 mc_reg_read "KSZ9567R" 0xN804 4 
mc_reg_read "KSZ9567R" 0xN808 4 
mc_reg_read "KSZ9567R" 0xN80C 1 
mc_reg_read "KSZ9567R" 0xN820 4 
mc_reg_read "KSZ9567R" 0xN824 4 
mc_reg_read "KSZ9567R" 0xN830 4 
mc_reg_read "KSZ9567R" 0xN834 4
 mc_reg_read "KSZ9567R" 0xN840 4 
mc_reg_read "KSZ9567R" 0xN844 4 
mc_reg_read "KSZ9567R" 0xN848 4
 mc_reg_read "KSZ9567R" 0xN900 4 
mc_reg_read "KSZ9567R" 0xN904 4 
mc_reg_read "KSZ9567R" 0xN914 1 
mc_reg_read "KSZ9567R" 0xN915 1
 mc_reg_read "KSZ9567R" 0xN916 2 
mc_reg_read "KSZ9567R" 0xN918 2 
mc_reg_read "KSZ9567R" 0xN91A 2 
mc_reg_read "KSZ9567R" 0xN920 1 
mc_reg_read "KSZ9567R" 0xN923 1 
mc_reg_read "KSZ9567R" 0xN924 4
 mc_reg_read "KSZ9567R" 0xNA00 4 
mc_reg_read "KSZ9567R" 0xNA04 4 
mc_reg_read "KSZ9567R" 0xNB00 1 
mc_reg_read "KSZ9567R" 0xNB01 1
 mc_reg_read "KSZ9567R" 0xNB04 1
 mc_reg_read "KSZ9567R" 0xNC00 2 
mc_reg_read "KSZ9567R" 0xNC02 2
 mc_reg_read "KSZ9567R" 0xNC04 2 
mc_reg_read "KSZ9567R" 0xNC08 2
 mc_reg_read "KSZ9567R" 0xNC0A 2 
mc_reg_read "KSZ9567R" 0xNC0C 2
 mc_reg_read "KSZ9567R" 0xNC0E 2 
mc_reg_read "KSZ9567R" 0xNC10 2 
mc_reg_read "KSZ9567R" 0xNC12 2 
mc_reg_read "KSZ9567R" 0xNC14 2
 mc_reg_read "KSZ9567R" 0xNC16 2 
mc_reg_read "KSZ9567R" 0xNC18 4 
mc_reg_read "KSZ9567R" 0x420 4 
mc_reg_read "KSZ9567R" 0x424 4 
mc_reg_read "KSZ9567R" 0x428 4 
mc_reg_read "KSZ9567R" 0x42C 4 
mc_reg_read "KSZ9567R" 0x420 4 
mc_reg_read "KSZ9567R" 0x424 4 
mc_reg_read "KSZ9567R" 0x428 4 
mc_reg_read "KSZ9567R" 0x42C 4 
mc_reg_read "KSZ9567R" 0x424 4 
mc_reg_read "KSZ9567R" 0x02 2 
mc_reg_read "KSZ9567R" 0x07 2




