
set -e
# 可以安装到thirdparty 中，不要安装到根目录中

cd /tmp
rm -rf PROJ

INSTALL_PREFIX="$CURRENT_PATH/../third_party/install/proj"

git clone --depth 1 --recursive --branch 6.0 git@github.com:OSGeo/PROJ.git


cd PROJ

sudo apt-get install sqlite3

mkdir build
cd build
# cmake .. -DBUILD_SHARED_LIBS=ON
cmake .. -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/mnt/easy_apollo/third_party/install/proj 

make -j8

sudo make install

sudo ldconfig
