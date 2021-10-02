#include "ReadParsePackage.h"
#include <stdio.h>
#include <chrono>

#define CENTER_BASE_ANGLE 28.5  //线性补偿角度

#define FPS_MAX 3100    //最大fps
#define FPS_MIN 2900    //最小fps
#define FPS_CONTINUE_SECOND 5    //fps异常的持续时间

#define SPEED_MAX 420     //最大速度
#define SPEED_MIN 300     //最小速度
#define SPEED_CONTINUE_SECOND 5   //最大速度或者最小速度异常的持续时间

ReadParsePackage::ReadParsePackage()
{
}

ReadParsePackage::~ReadParsePackage()
{

}

int ReadParsePackage::OpenSerial(char* port, unsigned int bauds)
{
    return serial_.openDevice(port, bauds);
}

void ReadParsePackage::CloseSerial()
{

    serial_.closeDevice();
}

int ReadParsePackage::ReadDataSerial(unsigned int TimeOut_ms)
{
    double duationMs = (GetTimeStamp() - startTimeStamp_) / 1.0e6;
    if (duationMs >= 1000.0)
    {
        data_rate_hz_ = data_num_total_;
        data_num_total_ = 0;
        startTimeStamp_ = GetTimeStamp();

        CheckInvalidFPS(data_rate_hz_);
    }

    int rtn;                                                // Returned value from Read
    bool found = false;
    int find_times = 0;
    int numDataByte;
    BYTE curByte = 0;
    BYTE preByte = 0;

    while (!found)
    {
        rtn = serial_.readChar(&curByte, TimeOut_ms);
        if (rtn == 1)                                                 // If a byte has been read
        {
            found = (preByte == BYTE(0x55)) && (curByte == BYTE(0xaa));
            numDataByte = 2;
            preByte = curByte;
        }
        else
        {
            SetReadCharsError(rtn);
            printf("k1\n");
            return m_lastErrorCode;
        }

        ++find_times;
        if (find_times >= 100)
        {
            // Find data header time out!
            m_lastErrorCode = ERR_FIND_HEAD_TIMEOUT;
            printf("k2\n");
            return m_lastErrorCode;
        }
    }

    return ReadSurplusData(TimeOut_ms);
}

int ReadParsePackage::ReadSurplusData(unsigned int TimeOut_ms)
{
    BYTE cInfo;
    int rtn = serial_.readChar(&cInfo, TimeOut_ms);
    if (rtn != 1)
    {
        SetReadCharsError(rtn);
        return -1;
    }

    switch (cInfo)
    {
    case 0x02:
    case 0x03:
    case 0x07:
        unitDataSize_ = GetDataByteLength(cInfo);
        return ReadLidarData(TimeOut_ms, cInfo);
    default:
        return -1;
    }

}

int ReadParsePackage::ReadLidarData(unsigned int TimeOut_ms, BYTE cInfo)
{
    BYTE buff[PACKAGESIZE] = { 0 };
    buff[0] = cInfo;

    //read data number
    int rtn = serial_.readChars(buff + 1, 1, TimeOut_ms);
    if (rtn != 1)
    {
        SetReadCharsError(rtn);
        return -1;
    }
    
    //data number
    data_num_per_pack_ = buff[1];
    
    int buff_length_target = data_num_per_pack_ * unitDataSize_ + 8;

    rtn = serial_.readChars(buff + 2, buff_length_target, TimeOut_ms);
    if (rtn != buff_length_target)
    {
        SetReadCharsError(rtn);
        return m_lastErrorCode;
    }

    bool is_equal = CheckLidarData(buff, buff_length_target + 2);
    if (!is_equal)
    {
        m_lastErrorCode = ERR_CHECKDATA_INVALID;
        return m_lastErrorCode;
    }

    memset(rcvbuffer_, 0, PACKAGESIZE);
    memcpy(rcvbuffer_, buff + 2, buff_length_target);

    data_num_total_ += data_num_per_pack_;

    return buff_length_target + 4;
}

bool ReadParsePackage::CheckLidarData(BYTE* buffer, int len)
{
    if (len / 2 == 1)
    {
        printf("it is error.\n");
        return false;
    }

    int* temp = new int[len / 2];//[17] = {0};
    //printf("len:%d\n", len / 2 );
    temp[0] = 0x55 + (0xAA << 8);
    for (int i = 1; i < len / 2; i++)
    {
        temp[i] = buffer[2 * (i - 1)] + (buffer[2 * (i - 1) + 1] << 8);
    }

    int chk32 = 0;
    for (int i = 0; i < len / 2; i++)
    {
        chk32 = (chk32 << 1) + temp[i];
    }

    int checksum_target = (chk32 & 0x7FFF) + (chk32 >> 15);
    checksum_target = checksum_target & 0x7FFF;
    int checksum_cur = buffer[len - 2] + (buffer[len - 1] << 8);
    bool is_equal = (checksum_target == checksum_cur);

    delete[] temp;
    return is_equal;
}

void ReadParsePackage::CheckInvalidFPS(int validNumber)
{
    if (validNumber < FPS_MIN || validNumber > FPS_MAX)
    {
        ++invalidFPSSecond_;
        if (invalidFPSSecond_ > FPS_CONTINUE_SECOND)
        {
            printf("FPS:%d\n", validNumber);
            m_lastErrorCode = ERR_LIDAR_FPS_INVALID;

            invalidFPSSecond_ = 0;
        }
    }
    else
    {
        invalidFPSSecond_ = 0;
    }
}

int64_t ReadParsePackage::GetTimeStamp()
{
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    std::chrono::nanoseconds ns = now.time_since_epoch();

    return ns.count();
}

void ReadParsePackage::CheckInvalidLowSpeed(unsigned int speed)
{
    if (speed < SPEED_MIN)
    {
        if (startTimeLowSpeed_ != 0)
        {
            int64_t endTime = GetTimeStamp();
            if ((endTime - startTimeLowSpeed_) / 1e9 >= SPEED_CONTINUE_SECOND)
            {
                m_lastErrorCode = ERR_LIDAR_SPEED_LOW;
                startTimeLowSpeed_ = 0;
            }
        }
        else
        {
            startTimeLowSpeed_ = GetTimeStamp();
        }
    }
    else
    {
        startTimeLowSpeed_ = 0;
    }
}

void ReadParsePackage::CheckInvalidHighSpeed(unsigned int speed)
{
    if (speed > SPEED_MAX)
    {
        if (startTimeHighSpeed_ != 0)
        {
            int64_t endTime = GetTimeStamp();
            if ((endTime - startTimeHighSpeed_) / 1e9 >= SPEED_CONTINUE_SECOND)
            {
                m_lastErrorCode = ERR_LIDAR_SPEED_HIGH;
                startTimeHighSpeed_ = 0;
            }
        }
        else
        {
            startTimeHighSpeed_ = GetTimeStamp();
        }
    }
    else
    {
        startTimeHighSpeed_ = 0;
    }
}

int ReadParsePackage::GetDataByteLength(char cInfo)
{
    switch (cInfo)
    {
    case 0x02:
        return 2;
    case 0x03:
        return 3;
    case 0x07:
        return 4;
    default:
        return -1;
    }
}

void ReadParsePackage::SetReadCharsError(int errCode)
{
    switch (errCode)
    {
    case -5:
        m_lastErrorCode = ERR_SERIAL_INVALID_HANDLE;
        break;
    case -4:
        m_lastErrorCode = ERR_SERIAL_SETCOMMTIMEOUTS_FAILED;
        break;
    case -3:
        m_lastErrorCode = ERR_GETPACKAGE_FAILED;
        break;
    case -1:
        m_lastErrorCode = ERR_SERIAL_READFILE_FAILED;
        break;
    case 0:
        m_lastErrorCode = ERR_SERIAL_READFILE_ZERO;
        break;
    }
}

int ReadParsePackage::ParseDataSerial(rangedata* dataPack, int& fps)
{
    ParseLidarData(rcvbuffer_, data_num_per_pack_, dataPack);

    fps = data_rate_hz_;

    return 0;
}

int ReadParsePackage::ParseLidarData(BYTE* in_buffer, const int in_numData, rangedata* out_dataPack)
{
    // The size of each data is 3 bytes
    int data_size = unitDataSize_;

    // in_numData: sampling number in every data package
    int id_start = 2;
    float FA = (in_buffer[id_start + 1] - 0xA0 + in_buffer[id_start] / 256.0) * 4;

    int id_LA_start = id_start + in_numData * data_size + 2;
    float LA = (in_buffer[id_LA_start + 1] - 0xA0 + in_buffer[id_LA_start] / 256.0) * 4;

    if (LA < FA) { LA += 360; }

    int len = in_numData - 1;
    if (len == 0)
    {
        return -1;
    }

    float dAngle = (LA - FA) / (in_numData - 1);        // angle info for each sampling

    unsigned char* data = new unsigned char[data_size];
    int pre_bytes = 4;          // 4 bytes before sampling data in each data package
    // calc speed (rpm)
    unsigned int speed = (in_buffer[1] << 8 | in_buffer[0]) / 64;

    CheckInvalidLowSpeed(speed);
    CheckInvalidHighSpeed(speed);

    double angle_offset = CENTER_BASE_ANGLE;
    for (int i = 0; i < in_numData; ++i)
    {
        //printf("%.4f\n", FA + dAngle * i);
        double angle_cur = FA + dAngle * i + angle_offset;
        if (angle_cur > 360)
        {
            angle_cur -= 360;
        }

        out_dataPack[i].angle = angle_cur;

        memcpy(data, in_buffer + pre_bytes + i * data_size, sizeof(unsigned char) * data_size);
        out_dataPack[i].flag = (data[1] >> 7) & 0x01;
        out_dataPack[i].dist = ((data[1] & 0x3F) << 8) | data[0];
        out_dataPack[i].speed = speed;
        if (data_size == 3)
        {
            out_dataPack[i].syn_quality = data[2];
        }
        else if (data_size == 4)
        {
            out_dataPack[i].syn_quality = (data[3] << 8) | data[2];
        }
        else
        {
            out_dataPack[i].syn_quality = 0;
        }

        if (0 == out_dataPack[i].dist)
        {
            out_dataPack[i].flag = true;
        }

    }

    delete[] data;
    return 0;
}