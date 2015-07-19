#! /bin/sh

ifdown eth0
ifconfig eth0 down hw ether 40:16:7e:af:ac:7f
ifup eth0
