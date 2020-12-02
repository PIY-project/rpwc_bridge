# rpwc_bridge
rpwc_bridge


POCO install step

wget https://pocoproject.org/releases/poco-1.8.1/poco-1.8.1.tar.gz

#Install dependences
sudo apt-get install openssl libssl-dev
sudo apt-get install libiodbc2 libiodbc2-dev
sudo apt-get install libmysqlclient-dev

cd poco-1.8.1
#sudo ./configure --omit=Data/ODBC,Data/MySQL --no-tests --no-samples --shared
cd build 
cmake-gui ..
sudo make -j8 
sudo make install 