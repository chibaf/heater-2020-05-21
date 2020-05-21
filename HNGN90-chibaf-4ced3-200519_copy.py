#!/usr/bin/env python3
""" HNGN001 Created on Tue Jun 11 08:34:11 2019
(two days before 190613) (1month before 190711)"""


s1="""HNGN1,澄江\n """

s2="""
2020年4月の517号室での最初の加熱実験では、加熱の入力が340ワット。それで200度から640度まで20秒で上がった。

HNGN82 から、温度上昇のモデルを引き継ぐ。
"""

s3="""
時間刻みの中で全てのストーリーが成り立つように組み上げる。
"""

print(s1+"\n"+s2+"\n"+s3+"\n")

s4="""
t = i*dt
T=Treal-Ttarget

T += dt*(Q/H)

W_heat=Input_of_Control_Loop=heater_power_input

W_heat += Kp*(T) + Kd*(Q/H) + Ki*(T*dt)
Q=W_heat-W_cool
"""

s5="""
{200502}
x <=> T
w <=> (Tdt)
v <=> (Q/H)

	for i in range(int(50/dt)):
		t = i*dt
		x += dt*v
		w += dt*x
		F = -x*Kp -v*Kd -w*Ki
		v += dt*F/m
"""

def W_cool():
	s=""" Cooling power (J/sec)
	refer HNGN82_200427
	H=heat capasiry(J/K)
	Troom=20
	Q=stored heat(J)
	"""
	Hcond=0.2
	W=Hcond*(dT+Ttarget-Troom)
	return W

import numpy as np
import serial
import time
import sys
import re
from pylab import *

# strPort = '/dev/tty.usbserial-A7006Yqh'
print ("pls input port name = M5Stck, ARDUINO, file_name.csv")
print ("ls /dev/*usb/*, and, ls /dev/*USB/*  =command to find port name") 

# plot parameters
#analogData = AnalogData(100)
#analogPlot = AnalogPlot(analogData)

#tc1 = [0] * 100
#tc2 = [0] * 100
#cnv = [0]*100
t=np.linspace(0.0,10.0,100)
# open serial port
strPort1 = sys.argv[1];
strPort2 = sys.argv[2];
file1=sys.argv[3];
ser1 = serial.Serial(strPort1, 115200) # M5Stack Serial Speed
ser2 = serial.Serial(strPort2, 115200) # Arduino Serial Speed
f=open(file1,"w+")
#
#
#fig, ax = plt.subplots()
ims = []
dt = 0.1 
Kp = 0.0			
Kd = 0.0
Ki = 0.0	

y=[0]*10
regex = re.compile('\d+')
data=[]
flag=0
Troom=20
W_heat=50 #=280,(J/sec) initial condition
itime=0
sum_error=0
Told=Troom
integral_error=0
while 1:
  try:
    dT_list = []
    t_list = []
    if Kp < 0.0:		#比例ゲイン
      Kp += 0.01
    elif Ki < 0.001:	#積分ゲイン
      Ki += 0.001
    elif Kd < 0.5:		#微分ゲイン
      Kd += 0.01
	
	#### ###				
    Q=Initial_heat=2000 #initial condition
    W_heat_max=300 #heater power at 100V (MAX)
    rCp=3.36  # =(J/K)from H of HNGN82
#     T=Q/rCp-273.15
    Ttarget=T_target_degC=300 #Target
    TtargetK=Ttarget+273.15 
    dQ=0

    line = ser1.readline() #read logger data
    print(line)
    match = regex.findall(str(line))
    data.append(itime*0.1)    # set time and temps to data
    data.append(float(match[4]+"."+match[5]))
    data.append(float(match[6]+"."+match[7]))
    data.append(float(match[8]+"."+match[9]))
    data.append(float(match[10]+"."+match[11]))
    data.append(float(match[12]+"."+match[13]))
    data.append(float(match[14]+"."+match[15]))
    data.append(float(match[16]+"."+match[17]))
    data.append(float(match[18]+"."+match[19]))
    data.append(float(match[20]+"."+match[21]))
    data.append(float(match[22]+"."+match[23]))
	
	#Tc thermo-couple-reading
    T=data[4] #unit=Cdegree) reading from Tc
    print (data)
    print("T=",T)
    dT=error_T=T-Ttarget
    integral_error+=error_T*dt
    diffT= T-Told
    Told = T
    print("dT=",dT)
    Q=(T+273.15)/rCp
    print("W_cool=",W_cool())
    print("W_heat=",W_heat)
    x=range(0, 10, 1)
    y.insert(0, T)
    y.pop(10)
    clf()
    ylim(0, 400)
    plot(x, y)
    pause(0.05)
#    show()
#    W_heat= -Kp*(rCp*dT)*dt -Kd*(dQ/dt)*dt -Ki*(dT*dt)
#    W_heat= -Kp*(rCp*dT) -Kd*(W_heat) -Ki*(W_heat)
#    diffTbydt= (Tnew-Told)/dt 
# integral[error]dt = integral(error_T*dt) =sum_error
    Kp=7.0; Ki=0; Kd=0
    W_heat= -Kp*dT -Ki*(integral_error)-Kd*(diffT/dt)
    print("W_heat=",W_heat)
    print("integral_error=", integral_error)
    print("diffT/dt      =",diffT/dt)
    data.append(W_heat)         # set PID terms to data
    data.append(dT)         
    data.append(integral_error)
    data.append(diffT/dt)
#    dQ=dt*(W_heat-W_cool())
#    dQ=dt*(W_heat)
#    print("dQ=",dQ,"\n")
#    Q +=dQ
#    T=Q/rCp-273.15
    if (W_heat > W_heat_max):
      val=255
    else:        
      val=int(W_heat/W_heat_max*255)%256		
    a = val.to_bytes(1, byteorder="little")
    ser2.write(a)
    for val in data:   # write out data to csv file
      flag+=1
      f.write(str(val)); 
      if(flag<15): 
        f.write(", ")
    f.write("\n")
    flag=0
    itime+=1
    data=[]

  except KeyboardInterrupt:
    print ('exiting')
    break

f.close()
ser1.flush()
ser1.close()
ser2.flush()
ser2.close()
