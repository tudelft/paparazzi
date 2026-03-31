# Function to display help
usage() {
    echo "====================================================="
    echo "Usage: $0 <options>"
    echo "Options:"
    echo "  -h, --help    Display this help message"
    echo ""
    echo "WARNING:"
    echo "  This script is run on the terminal when the drone"
    echo "  is on, BUT NOT IN FLIGHT. This script restarts the"
    echo "  autopilot, so it will also turn off the motors."
    echo ""
    echo "NOTE:"
    echo "  If the terminal says 'permission denied', run this"
    ehco "  command:    chmod +x ap_restart.sh"
    echo "====================================================="
    exit 0
}

# Check for help flag
if [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    usage
fi

# Kill the currently running autopilot process 
echo "Autopilot: killing ap.elf..."
killall -9 ap.elf 

# Navigate to the Paparazzi directory 
cd /data/ftp/internal_000/paparazzi/ 

# Start the autopilot process manually 
echo "Autopilot: restarting ap.elf..."
./ap.elf
