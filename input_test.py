import threading
import argparse

def get_input():
    angle0 = float(input())
    

    
    return angle0



for i in range(10):
    if i%2==0:
        print("type : ")
        input_thread = threading.Thread(target=get_input)
        input_thread.start()


