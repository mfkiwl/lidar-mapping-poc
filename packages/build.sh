root=$(pwd)

echo "Build SerialPort"

cd SerialPort
make clean
make
sudo make install

cd $root

echo "Build RF24"

cd RF24
./configure --driver=SPIDEV
make clean
make
sudo make install

cd $root

echo "Build OLED SSD1306"

cd OLED_SSD1306_I2C_Linux
make clean
make
sudo make install

cd $root
