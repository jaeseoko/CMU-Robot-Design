import pyvisa as visa
import time
from time import sleep
import numpy as np


rm = visa.ResourceManager()
print('Connected VISA resources:')
print(rm.list_resources())

dmm = rm.open_resource('USB0::0x1AB1::0x0C94::DM3O171700266::INSTR')
dmm2 = rm.open_resource('USB0::0x1AB1::0x0C94::DM3O191900311::INSTR')
# print('Instrument ID (IDN:) = ', dmm.query('*IDN?'))
#print("Volts DC   = ", dmm.query(":MEASure:VOLTage:DC?"))
# print("DC Current = ", dmm.query(":MEASure:CURRent:DC?"))
print(" Seconds Count    ", "DC Current", "Raw Meter Response", sep="\t|\t")
print("----------------------------------------------------------------------------------")

start_time = time.time()


data_points = 20


current_data = np.zeros((data_points, 2))
current_data2 = np.zeros((data_points, 2))

for x in range(0, data_points):
    rawStr = dmm.query(":MEASure:CURRent:DC?")

    iStr = rawStr
    rawStr = rawStr.replace ("\n", "") 
    iStr = iStr.replace("\n", "")
    iStr = iStr.replace("#9000000015", "")

    iFlt = float(iStr)

    rawStr2 = dmm2.query(":MEASure:CURRent:DC?")
    # print(rawStr)

    iStr2 = rawStr2
    rawStr2 = rawStr2.replace ("\n", "") 
    iStr2 = iStr2.replace("\n", "")
    iStr2 = iStr2.replace("#9000000015", "")

    iFlt2 = float(iStr2)


    now = time.time() - start_time

    current_data[x][0] = now
    current_data[x][1] = iFlt

    current_data2[x][0] = now
    current_data2[x][1] = iFlt2

    print(now, iFlt ,rawStr, sep="\t|\t")
    print(now, iFlt2 ,rawStr, sep="\t|\t")

    # sleep(.01)

np.savetxt("currentData.csv", current_data, delimiter=',', header="Time (Seconds),Current (Amps)", comments="")
np.savetxt("currentData2.csv", current_data2, delimiter=',', header="Time (Seconds),Current (Amps)", comments="")
