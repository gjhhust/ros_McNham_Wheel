#include "ReadParsePackage.h"

#include <thread>
#include <mutex>
#include <cmath>
#include <list>

struct node_info
{
    uint16_t    syn_quality;
    uint16_t   angle_q6_checkbit;
    uint16_t   distance_q2;
    uint16_t   isValid;// 1有效  0 无效
    bool operator<(const node_info& _Left) const
    {
        if (_Left.angle_q6_checkbit < this->angle_q6_checkbit)
        {
            return false;
        }

        return true;
    }
};

class Dev
{
public:
    Dev();
    ~Dev();
    
    //打包数据的所有接口 步骤：初始化，获取数据，结束

    //isGetLoopData：true,表示获取一圈数据再输出；false,表示数据实时输出。默认是一圈输出
    //返回值:0，表示成功  非零表示失败
    int Initialize(char* port, unsigned int baud, bool isGetLoopData = true);
    //非阻塞  返回值:0，表示成功; 非零表示失败
    
    //需要判断count是否获取到数据
    void GetScanData(node_info * nodebuffer,size_t buffLen, size_t &count, bool isReverse = true);
	void GetScanData(std::list<node_info>& dataList, bool isReverse = true);

    int Uninit();
    //---------------------------------------打包数据的所有接口

    //获取错误码
    int GetLastErrCode()
    {
        return rdPrPack_.GetLastErrCode();
    }

private:

    //isGetLoopData is false
    void PushDataWithNoLoopMode(node_info& node_cur);

    //isGetLoopData is true
    void PushDataWithLoopMode(bool& isTurn, std::list<node_info>& loopNodeList, node_info& node_cur);

    void CacheScanData();
    
    void GrabScanData(node_info* nodebuffer, size_t buffLen, size_t& count);
	void GrabScanData(std::list<node_info>& dataList);

    void GrabScanDataWithNoLoop(std::list<node_info>& nodeList, node_info* nodebuffer, size_t buffLen);

    void GrabScanDataWithLoop(std::list<node_info>& nodeList, node_info* nodebuffer, size_t buffLen);
    void PushValidData2Buffer(node_info& nodeInfo, int index, node_info* nodebuffer, int len);
    bool CheckBufferIsSorted(node_info* nodebuffer, int len);

    void CheckInvalidLidarNumber(int validNumber);
    
    std::list<node_info> m_nodeList;
    
    bool is_scanning_;
    
    std::thread thread_;
    std::mutex mutex_;

    bool isGetLoopData_;
    int invalidNumberContinue_ = 0;

    ReadParsePackage rdPrPack_;
	bool isGreaterThan = false;

};
