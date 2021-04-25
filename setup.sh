pip3 install pigpio;
pip3 install numpy-stl;
pip3 install svgpathtools;
pip3 install scipy;
sudo apt-get install libatlas-base-dev;
pip3 install matplotlib;
sudo apt-get install python3-pyqt5;

sudo apt-get update;
sudo apt-get install curl;
sudo apt-get install g++;
sudo apt-get install make;

curl -L http://download.osgeo.org/libspatialindex/spatialindex-src-1.8.5.tar.gz | tar xz;
cd spatialindex-src-1.8.5;
./configure;
make;
sudo make install;
sudo ldconfig;

sudo apt-get install libgeos-dev;
pip3 install trimesh;
pip3 install shapely;
pip3 install networkx;
#pip3 install pyembree;
pip3 install pyglet;
#pip3 install descartes; # also edit $PYTHONPATH/site-packages/descartes/__init__.py and add PatchPath to imports
pip3 install lxml;
pip3 install svg.path;
pip3 install Rtree

#new config using conda:
conda install -c cbetters pigpio
conda install -c conda-forge numpy-stl
conda install -c conda-forge pyyaml
