#!/bin/bash

HYSTRIX_PM="hystrix-pm "
HYSTRIX_VOC="hystrix-voc "
HYSTRIX_EC="hystrix-ec "
HYSTRIX_HUMTEMP="/sys/class/hwmon/hwmon1"
GPIO_BASE="/sys/class/gpio"

SEL_SENS=({0..1}{0..1}{0..1})
GPIO_SEL_SENS=("19" "17" "16")

reserve_gpio (){
  # gpio sel_sens 0-2 
  for i in 16 17 19
  do
    echo ${i}> ${GPIO_BASE}/export
    echo out > ${GPIO_BASE}/gpio${i}/direction
  done 

  # gpio code 0-2   
  for i in 21 20 18
  do
    echo ${i}> ${GPIO_BASE}/export
    echo in > ${GPIO_BASE}/gpio${i}/direction
  done
}

release_gpio (){
  for i in {16..21}
  do
    echo ${i}> ${GPIO_BASE}/unexport
  done
}

read_code(){
  local GPIO_CODE=()
  
  #reversed order
  for i in 18 20 21
  do
    GPIO_CODE=(${GPIO_CODE[@]} `cat ${GPIO_BASE}/gpio${i}/value`)
  done
  CHAN_CODE=`printf %s "${GPIO_CODE[@]}" $'\n'`
}

read_humidity (){
  CHAN_HUM=`cat ${HYSTRIX_HUMTEMP}/humidity1_input`
}

read_temperature () {
  CHAN_TEMP=`cat ${HYSTRIX_HUMTEMP}/temp1_input` 
}

read_channel () {
  # param 1 - channel number

  local CHAN_STATUS="true"

  # mux gpio decode
  local MUX_SEL_VAL=${SEL_SENS[${1}]}

  # set mux 
  for i in {0..2}
  do
    echo ${MUX_SEL_VAL:${i}:1} > ${GPIO_BASE}/gpio${GPIO_SEL_SENS[${i}]}/value
  done

  #read sensor
  case ${1} in
    [0-3]) eval "${HYSTRIX_EC} ${HYSTRIX_SERVER_SOCKET} ${1}" ;;
    [4-7]) eval "${HYSTRIX_VOC} ${HYSTRIX_SERVER_SOCKET} ${1}" ;;
  esac

  CHAN_READING=$?
  if test ${CHAN_READING} -ne 0
  then
    CHAN_STATUS="false"
  fi
  echo "chan ${1} - read sensor ${CHAN_READING}"

  #read code
  local EC_CODE="\"code\":9,"

  case ${1} in
    [0-3]) 
          read_code
          EC_CODE="\"code\":$((2#${CHAN_CODE})),"
          ;;
  esac

  #load hwmon
  echo sht21 0x40 > /sys/bus/i2c/devices/i2c-2/new_device

  #discard first hum/temp reading
  echo "chan ${1} discard: `cat ${HYSTRIX_HUMTEMP}/humidity1_input`"
  echo "chan ${1} discard: `cat ${HYSTRIX_HUMTEMP}/temp1_input`"

  #read humidity
  read_humidity

  if [[ ${CHAN_HUM} -eq "" ]]
  then
    CHAN_STATUS="false"
    CHAN_HUM="0"
  else
      echo "chan ${1} - read humidity : ${CHAN_HUM}"
  fi

  #read temperature
  read_temperature

  if [[ ${CHAN_TEMP} -eq "" ]]
  then
    CHAN_STATUS="false"
    CHAN_TEMP="0"
  else
      echo "chan ${1} - read temperature: ${CHAN_TEMP}"      
  fi

  #unload hwmon
  echo 0x40 > /sys/bus/i2c/devices/i2c-2/delete_device

  # notify server
  echo "{\"chan\":${1},${EC_CODE}\"hum\":${CHAN_HUM},\"temp\":${CHAN_TEMP},\"status\":${CHAN_STATUS}}" | socat UNIX-CONNECT:${HYSTRIX_SERVER_SOCKET} -
}

read_pm_channel () {
  eval "${HYSTRIX_PM} ${HYSTRIX_SERVER_SOCKET} ${1}"  
  CHAN_READING=$?
  echo "chan 8 - read sensor ${CHAN_READING}"
  if test ${CHAN_READING} -ne 0
  then
    CHAN_STATUS="false"
  else
    CHAN_STATUS="true"
  fi
  echo "{\"chan\":${1},\"code\":8,\"status\":${CHAN_STATUS}}" | socat UNIX-CONNECT:${HYSTRIX_SERVER_SOCKET} -
}

# main sequence
#
# 1) reserve gpios
# 2) channels iteration
# 3) read val from channel
# 4) read temperature/humidity for channel
# 5) notify server

HYSTRIX_SERVER_SOCKET=${1}

# 1) reserve gpios
reserve_gpio

#2) channel iteration
for chanNum in {0..7}
do
  read_channel ${chanNum}
done

# read pm
read_pm_channel 8

release_gpio
exit 0
