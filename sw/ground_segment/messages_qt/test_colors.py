import subprocess
import time
import os
os.environ['QT_QPA_PLATFORMTHEME'] = 'gtk3'
os.environ['GTK_THEME'] = 'Adwaita-dark'
# We just want to launch it and see if it crashes or prints any QPalette warnings.
p = subprocess.Popen(["./build/messages_qt"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
time.sleep(2)
p.kill()
stdout, stderr = p.communicate()
print("STDOUT:", stdout.decode())
print("STDERR:", stderr.decode())
