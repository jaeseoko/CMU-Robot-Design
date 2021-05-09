import pyvisa as visa
import time
from time import sleep
import numpy as np

rm = visa.ResourceManager()
print('Connected VISA resources:')
print(rm.list_resources())

dmm = rm.open_resource('USB0::0x1AB1::0x0C94::DM3O191900311::INSTR')
print('Instrument ID (IDN:) = ', dmm.query('*IDN?'))
#print("Volts DC   = ", dmm.query(":MEASure:VOLTage:DC?"))
print("DC Current = ", dmm.query(":MEASure:CURRent:DC?"))

print("Poll rate = 500mS. Will run for 24 hours collecting 172,800 readings")
print("output file = iLog.csv\n\n")
print(" Seconds Count    ", "DC Current", "Raw Meter Response", sep="\t|\t")
print("----------------------------------------------------------------------------------")

start_time = time.time()

for x in range(0, 100):
    rawStr = dmm.query(":MEASure:CURRent:DC?")
    iStr = rawStr
    rawStr = rawStr.replace ("\n", "") 
    iStr = iStr.replace("\n", "")
    iStr = iStr.replace("#9000000015", "")

    iFlt = float(iStr)
    now = time.time() - start_time

    current_data[x][0] = now
    current_data[x][1] = iFlt

    print(now, iFlt ,rawStr, sep="\t|\t")

    sleep(.03)

numpy.savetxt("currentData.csv", a, delimiter=',', header="A,B", comments="")
