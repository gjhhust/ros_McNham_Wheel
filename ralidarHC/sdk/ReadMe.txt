Version: 1.8.2

Tutorial:
1. Compile command (Need compiler which supports C++11)
    1) Windows
    mkdir build
    cd build
    cmake -G "Visual Studio 11 2012" ..
    (Notice: please change the cmake command if you use other compiler)
    
    2) Linux 
    mkdir build
    cd build
    cmake ..
    make
    
2. Modify port number
    1) Use the USB port number which the lidar is connected to. Please check code and comments for details.
    2) (For Linux) After the USB port number is modified, execute make command to generate all the *.exe files.
    
3. Description of print statements 
    1) sdk_demo.exe (Output single data package of lidar)
    
    printf("%d, dist: %d, angle: %f, is_invalid: %d\n", i, dataPack[i].dist, dataPack[i].angle, dataPack[i].flag);
    
    From left to right:
    i           -> index number of data package (0-7)
    dist        -> distance info (unit: mm)
    angle       -> angle info (in degrees)
    is_invalid  -> validity of current data package (0: valid, 1: invalid)
    
    2) sdk_scanData.exe (Output single-turn data package of lidar)
    
    printf("data size=%d, FD=[%.2f], LD=[%.2f], TC=%.3fms, stamp=%llu\n",
        count,
        nodebuffer[0].angle_q6_checkbit / 64.0f,
        nodebuffer[count-1].angle_q6_checkbit / 64.0f,
        timer.Duation_ms(),
        data_stamp_new);
    
    From left to right:
    data size -> total number of single-turn data packages (for invalid data package, the distance value is set to 0)
    FD        -> first angle info of single-turn data packages
    LD        -> last angle info of single-turn data packages
    TC        -> time cost of single-turn data packages
    stamp     -> data stamp of single-turn data packages (increasing by 1)