# f110-mpc
model predictive control in f1tenth simulator
![mpc](https://user-images.githubusercontent.com/75038294/166095543-be81a7df-74e3-492a-a502-edbda07f206d.png)

conda install -c conda-forge xtensor
git clone git@github.com:rjsberry/xtensor-interpolate.git

# osqp
git clone --recursive https://github.com/oxfordcontrol/osqp
cd osqp
mkdir build
cd build
cmake -G "Unix Makefiles" ..
cmake --build .
cmake --build . --target install

# osqp-eigen
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen
mkdir build && cd build
cmake ../
make
sudo make install
