#!/usr/bin/env python3
import os
import time

def exists(path):
    """Test whether a path exists.  Returns False for broken symbolic links"""
    try:
        os.stat(path)
    except OSError:
        return False
    return True
             
while (True):
     print(exists("/dev/usb/hiddev0"))
     time.sleep(1)



