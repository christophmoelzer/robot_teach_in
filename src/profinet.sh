#!/bin/bash

echo START_PROFINET
sleep 5
read -s -p "Enter password for sudo: " sudoPW
echo $sudoPW | sudo -S /home/chris/profinet/build/pn_dev -i enx806d97057607 -v -v -v -v