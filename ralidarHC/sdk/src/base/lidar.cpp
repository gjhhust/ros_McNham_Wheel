#include "lidar.h"
#include <vector>
#include <set>
#include <chrono>
#include <algorithm>

#ifndef PI
#define PI 3.141592653589793
#endif

#define CENTER_BASE_LINE 17.9   //非线性补偿时的基准距离

#define VALID_NUMBER_COUNT 50    //一圈有效点数的阈值
#define NUMBER_CONTINUE_CIRCLE 50    //一圈有效点数异常的持续圈数

//在判断是否是一圈出现异常情况时【即角度未出现反转】，该参数生效
#define CIRCLE_NUMBER_MAX 515  //注意:2000转设置415  3000转设置515

#define LESS_THAN_NUMBER 32 //反转出现少于32个点丢弃


Dev::Dev()
{
}

Dev::~Dev()
{
}

//isGetLoopData 是否输出一圈的数据
int Dev::Initialize(char* port, unsigned int baud, bool isGetLoopData /*false*/)
{
    if (rdPrPack_.OpenSerial(port, baud) != 1)
    {
        printf("open serial failed.\n");
        return -1;
    }

    is_scanning_ = true;
    isGetLoopData_ = isGetLoopData;

    thread_ = std::thread(&Dev::CacheScanData, this);

    return 0;
}

void Dev::PushDataWithNoLoopMode(node_info& node_cur)
{
    std::lock_guard<std::mutex> guard(mutex_);
    if (m_nodeList.size() >= 2048)
    {
        m_nodeList.pop_front();
    }

    m_nodeList.push_back(node_cur);
}

//isGetLoopData is false
void Dev::PushDataWithLoopMode(bool& isTurn, std::list<node_info>& loopNodeList, node_info& node_cur)
{
	if (isTurn)
	{
		if (!isGreaterThan || (isGreaterThan && loopNodeList.size() > LESS_THAN_NUMBER))
		{
			std::lock_guard<std::mutex> guard(mutex_);
			m_nodeList.clear();
			m_nodeList.swap(loopNodeList);
		}
		else
		{
			loopNodeList.clear();
		}

		isTurn = false;
		isGreaterThan = false;

	}
	else if (loopNodeList.size() >= CIRCLE_NUMBER_MAX)
	{
		{
			std::lock_guard<std::mutex> guard(mutex_);
			m_nodeList.clear();
			m_nodeList.swap(loopNodeList);
		}

		isGreaterThan = true;
	}
	loopNodeList.push_back(node_cur);
}

void Dev::CacheScanData()
{
    int fps = 0, rtn = 0;
    
    double angle_cur = 0;
    //double m = 20;
    double preAngle = 0;
    bool isTurn = false;
    std::list<node_info> loopNodeList;
    bool isFirsLoop = false;
	int sameAngleCount;
    int validNumber = 0;
    rdPrPack_.SetStartTimestamp();
    while (is_scanning_)
    {
        rtn = rdPrPack_.ReadDataSerial();                    // Read data from serial port
        if (rtn <= 0)
        {
            printf("Warning: Read serial data time out! %d\n", rtn);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        rangedata* dataPack = new rangedata[rdPrPack_.data_num_per_pack_];
        rdPrPack_.ParseDataSerial(dataPack, fps);        // Parse data

        for (int i = 0; i < rdPrPack_.data_num_per_pack_; ++i)
        {
            node_info node_cur;
            angle_cur = dataPack[i].angle;
            // Compensate angle & dist
            unsigned int dist = dataPack[i].dist;

            if (0 != dist && !dataPack[i].flag)//&& !dataPack[i].flag
            {
                //printf("%.4f\n", atan(dist / m));

                double angle_correct_cur = angle_cur + 80.0 - atan(dist / CENTER_BASE_LINE) / PI * 180;
                if (angle_correct_cur > 360)
                {
                    angle_correct_cur -= 360;
                }
                else if (angle_correct_cur < 0)
                {
                    angle_correct_cur += 360;
                }

                dataPack[i].angle = angle_correct_cur;
                dataPack[i].dist = sqrt(CENTER_BASE_LINE * CENTER_BASE_LINE + 1.0 * dist * dist);
            }

            if (!dataPack[i].flag)
            {
                // valid data
                node_cur.distance_q2 = (uint16_t)(dataPack[i].dist * 4);

                node_cur.isValid = 1;
                ++validNumber;
            }
            else
            {
                // invalid data
                node_cur.distance_q2 = 0;
                node_cur.isValid = 0;
                //dataPack[i].angle = 0;
            }

            node_cur.angle_q6_checkbit = (uint16_t)(dataPack[i].angle * 64);
            node_cur.syn_quality = dataPack[i].syn_quality;

            if (preAngle > angle_cur)
            {
                isTurn = true;
                isFirsLoop = true;

                CheckInvalidLidarNumber(validNumber);
                validNumber = 0;
            }
            //printf("%.4f,%.4f\n", preAngle, angle_cur);
            if (!isFirsLoop)
            {
                preAngle = angle_cur;
                continue;
            }

            if (!isGetLoopData_)
            {
                PushDataWithNoLoopMode(node_cur);
            }
            else
            {
                PushDataWithLoopMode(isTurn, loopNodeList, node_cur);
            }

            preAngle = angle_cur;
            //printf("%.3f\n", node_cur.angle_q6_checkbit /  64.0f);
        }
        
        delete[] dataPack;
    } // while()
}

void Dev::GrabScanDataWithLoop(std::list<node_info>& nodeList, node_info* nodebuffer, size_t buffLen)
{
    int len = nodeList.size();
    
    double perAngle = 360.0 / nodeList.size();
    for (int i = 0; i < len; ++i)
    {
        nodebuffer[i].angle_q6_checkbit = perAngle * (i + 1.0) * 64;
        nodebuffer[i].distance_q2 = 0;
        nodebuffer[i].isValid = 0;
        nodebuffer[i].syn_quality = 0;
        //printf("%.3f,%d,%d\n", nodebuffer[i].angle_q6_checkbit, nodebuffer[i].distance_q2, nodebuffer[i].isValid);
    }

    for (auto it = nodeList.begin(); it != nodeList.end(); ++it)
    {
        if (it->isValid == 1)
        {
            int index = (it->angle_q6_checkbit / (perAngle * 64.0)) + 0.5 - 1;
            //printf("index:%d\n", index);
            PushValidData2Buffer(*it, index, nodebuffer, len);
        }
    }

    if (!CheckBufferIsSorted(nodebuffer, len))
    {
        //printf("data is not sorted.\n");
        std::stable_sort(nodebuffer, nodebuffer + len);
    }
}

void Dev::PushValidData2Buffer(node_info& nodeInfo, int index, node_info* nodebuffer, int len)
{
    bool isExit = false;
    while (!isExit)
    {
        index = index % len;
        if (nodebuffer[index].isValid == 0)
        {
            nodebuffer[index] = nodeInfo;
            isExit = true;
        }

        ++index;
    };
}

bool Dev::CheckBufferIsSorted(node_info* nodebuffer, int len)
{
    if (len < 2)
    {
        return true;
    }

    for (int i = 1; i < len; ++i)
    {
        if (nodebuffer[i - 1].angle_q6_checkbit > nodebuffer[i].angle_q6_checkbit)
        {
            return false;
        }
    }

    return true;
}

void Dev::GrabScanDataWithNoLoop(std::list<node_info>& nodeList, node_info* nodebuffer, size_t buffLen)
{
    size_t index = 0;
    size_t indexCount = 0;
    size_t startIndex = 0;
    size_t nodeLen = nodeList.size();
    if (nodeLen > buffLen)
    {
        startIndex = nodeLen - buffLen;
    }

    for (auto it = nodeList.begin(); it != nodeList.end(); ++it, ++indexCount)
    {
        if (indexCount >= startIndex)
        {
            nodebuffer[index] = *it;
            ++index;
        }
    }
}

void Dev::GrabScanData(node_info * nodebuffer, size_t buffLen, size_t &count)
{
    std::list<node_info> nodeList;
    {
        std::lock_guard<std::mutex> guard(mutex_);
        nodeList.swap(m_nodeList);
    }

    count = nodeList.size();
    if (count == 0)
    {
        return;
    }

    if (isGetLoopData_)
    {
        if (count > buffLen)
        {
            printf("buffer is too small");
            //m_lastErrorCode = ERR_RECEIVE_BUFFER_SMALL;
            rdPrPack_.SetLastErrCode(ERR_RECEIVE_BUFFER_SMALL);
            count = 0;
            return;
        }

        GrabScanDataWithLoop(nodeList, nodebuffer, buffLen);
    }
    else
    {
        GrabScanDataWithNoLoop(nodeList, nodebuffer, buffLen);
    }

}

void Dev::GrabScanData(std::list<node_info>& dataList)
{
    std::list<node_info> nodeList;
    {
        std::lock_guard<std::mutex> guard(mutex_);
        nodeList.swap(m_nodeList);
    }

    if (nodeList.empty())
    {
        return;
    }

    size_t count = nodeList.size();
    node_info* nodebuffer = new node_info[count];
    if (isGetLoopData_)
    {
        GrabScanDataWithLoop(nodeList, nodebuffer, count);
    }
    else
    {
        GrabScanDataWithNoLoop(nodeList, nodebuffer, count);
    }

    for (int i = 0; i < count; ++i)
    {
        dataList.push_back(nodebuffer[i]);
    }

    delete[] nodebuffer;

}

void Dev::GetScanData(node_info * nodebuffer, size_t buffLen, size_t &count, bool isReverse)
{
    if (nodebuffer == nullptr || buffLen == 0)
    {
        count = 0;
        return;
    }

    GrabScanData(nodebuffer, buffLen, count);

    for (int i = 0; i < count; ++i)
    {
        double angle_cur = nodebuffer[i].angle_q6_checkbit;
        angle_cur = angle_cur / 64.0;
        if (angle_cur > 360)
        {
            angle_cur -= 360;
        }
        else if (angle_cur < 0)
        {
            angle_cur += 360;
        }

        if (isReverse)
        {
            angle_cur = 360 - angle_cur;
        }

        nodebuffer[i].angle_q6_checkbit = angle_cur * 64;
    }
}

void Dev::GetScanData(std::list<node_info>& dataList, bool isReverse)
{
    GrabScanData(dataList);

    for (auto it = dataList.begin(); it != dataList.end(); ++it)
    {
        double angle_cur = it->angle_q6_checkbit;
        angle_cur = angle_cur / 64.0;
        if (angle_cur > 360)
        {
            angle_cur -= 360;
        }
        else if (angle_cur < 0)
        {
            angle_cur += 360;
        }

        if (isReverse)
        {
            angle_cur = 360 - angle_cur;
        }

        it->angle_q6_checkbit = angle_cur * 64;
    }
}

int Dev::Uninit()
{
    if(!is_scanning_)
        return 0;
    is_scanning_ = false;
    thread_.join();

    rdPrPack_.CloseSerial();

    return 0;
}

void Dev::CheckInvalidLidarNumber(int validNumber)
{
    if (validNumber < VALID_NUMBER_COUNT)
    {
        ++invalidNumberContinue_;
        if (invalidNumberContinue_ >= NUMBER_CONTINUE_CIRCLE)
        {
            //m_lastErrorCode = ERR_LIDAR_NUMBER_INVALID;
            rdPrPack_.SetLastErrCode(ERR_LIDAR_NUMBER_INVALID);
            invalidNumberContinue_ = 0;
        }
    }
    else
    {
        invalidNumberContinue_ = 0;
    }
}
