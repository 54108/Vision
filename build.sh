#!/bin/bash

### BEGIN INIT INFO
# Provides:          watchDog
# Required-Start:    $remote_fs $syslog
# Required-Stop:     $remote_fs $syslog
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: start watchDog
# Description:       start watchDog
### END INIT INFO
echo "606root" | sudo -S sudo rm -rf build/

mkdir build

cd build/

echo "606root" | sudo -S sudo cmake ..

echo "606root" | sudo -S sudo make -j

#echo "606root" | sudo -S sudo ./bin/WolfVision

exit 0

