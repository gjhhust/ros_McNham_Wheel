/*!
\file    serial-O-matic.cpp
\brief   Source file of the class HC_serial. This class is used for communication over a serial device.
\author  Philippe Lucidarme (University of Angers)
\version 1.2
\date    28 avril 2011

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE X CONSORTIUM BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


This is a licence-free software, it can be used by anyone who try to build a better world.
*/

#include "HC_serial.h"


#ifdef __linux__
#include <errno.h>
#include <mutex>
#include <vector>
#include <sys/signal.h>
#include <linux/serial.h>
#include "hchead.h"

int g_fd = 0;

struct sigaction g_saio;
struct timeval g_tv;

char g_u8ReadBuff[128];

std::mutex           g_mtxBuff;
std::vector<UCHAR>   g_lstRx;

void signal_handler_IO(int status)
{
	//int nread = read(g_fd, rbuff, 64);
	//if (nread > 0)
		//printf("%.*s", nread, rbuff);
	//LOG_INFO("signal_handler_IO\n");

	int nfds;
	int nread = 0;

	fd_set readfds;
	

	FD_ZERO(&readfds);
	FD_SET(g_fd, &readfds);

	nfds = select(g_fd + 1, &readfds, NULL, NULL, &g_tv);

	if (nfds == 0)
	{
		return;
	}
	else
	{
		nread = read(g_fd, g_u8ReadBuff, 128);
		//LOG_INFO("signal_handler_IO read=%d\n", nread);
		std::lock_guard<std::mutex> lock(g_mtxBuff);
		for (int i = 0; i < nread; i++)
		{
			g_lstRx.push_back(g_u8ReadBuff[i]);
		}
	}


}

#endif

#include <chrono>
//_____________________________________
// ::: Constructors and destructors :::


/*!
\brief      Constructor of the class HC_serial.
*/
HC_serial::HC_serial()
{
#if defined (_WIN32) || defined( _WIN64)
	hSerial = INVALID_HANDLE_VALUE;
#endif
#ifdef __linux__

	m_fd = 0;


#endif
}


/*!
\brief      Destructor of the class HC_serial. It close the connection
*/
// Class desctructor
HC_serial::~HC_serial()
{
    closeDevice();
}



//_________________________________________
// ::: Configuration and initialization :::



/*!
\brief Open the serial port
\param Device : Port name (COM1, COM2, ... for Windows ) or (/dev/ttyS0, /dev/ttyACM0, /dev/ttyUSB0 ... for linux)
\param Bauds : Baud rate of the serial port.

\n Supported baud rate for Windows :
- 110
- 300
- 600
- 1200
- 2400
- 4800
- 9600
- 14400
- 19200
- 38400
- 56000
- 57600
- 115200
- 128000
- 256000

\n Supported baud rate for Linux :\n
- 110
- 300
- 600
- 1200
- 2400
- 4800
- 9600
- 19200
- 38400
- 57600
- 115200

\return 1 success
\return -1 device not found
\return -2 error while opening the device
\return -3 error while getting port parameters
\return -4 Speed (Bauds) not recognized
\return -5 error while writing port parameters
\return -6 error while writing timeout parameters
*/
char HC_serial::openDevice(const char *chPort,const unsigned int iBauds)
{
#if defined (_WIN32) || defined( _WIN64)

    // Open serial port
    hSerial = CreateFileA(chPort,GENERIC_READ | GENERIC_WRITE,0,0,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,0);
    if(hSerial==INVALID_HANDLE_VALUE) {
        if(GetLastError()==ERROR_FILE_NOT_FOUND)
            return -1;                                                  // Device not found
        return -2;                                                      // Error while opening the device
    }

    // Set parameters
    DCB dcbSerialParams = {0};                                          // Structure for the port parameters
    dcbSerialParams.DCBlength=sizeof(dcbSerialParams);
    if (!GetCommState(hSerial, &dcbSerialParams))                       // Get the port parameters
        return -3;                                                      // Error while getting port parameters
    switch (iBauds)                                                      // Set the speed (Bauds)
    {
    case 110  :     dcbSerialParams.BaudRate=CBR_110; break;
    case 300  :     dcbSerialParams.BaudRate=CBR_300; break;
    case 600  :     dcbSerialParams.BaudRate=CBR_600; break;
    case 1200 :     dcbSerialParams.BaudRate=CBR_1200; break;
    case 2400 :     dcbSerialParams.BaudRate=CBR_2400; break;
    case 4800 :     dcbSerialParams.BaudRate=CBR_4800; break;
    case 9600 :     dcbSerialParams.BaudRate=CBR_9600; break;
    case 14400 :    dcbSerialParams.BaudRate=CBR_14400; break;
    case 19200 :    dcbSerialParams.BaudRate=CBR_19200; break;
    case 38400 :    dcbSerialParams.BaudRate=CBR_38400; break;
    case 56000 :    dcbSerialParams.BaudRate=CBR_56000; break;
    case 57600 :    dcbSerialParams.BaudRate=CBR_57600; break;
    case 115200 :   dcbSerialParams.BaudRate=CBR_115200; break;
    case 128000 :   dcbSerialParams.BaudRate=CBR_128000; break;
	case 153600:   dcbSerialParams.BaudRate = 153600; break;
    case 230400:   dcbSerialParams.BaudRate = 230400; break;
    case 256000 :   dcbSerialParams.BaudRate=CBR_256000; break;
    case 1500000 :   dcbSerialParams.BaudRate=1500000; break;
    default : return -4;
    }
    dcbSerialParams.ByteSize=8;                                         // 8 bit data
    dcbSerialParams.StopBits=ONESTOPBIT;                                // One stop bit
    dcbSerialParams.Parity=NOPARITY;                                    // No parity
    if(!SetCommState(hSerial, &dcbSerialParams))                        // Write the parameters
        return -5;                                                      // Error while writing

    // Set TimeOut
    timeouts.ReadIntervalTimeout=0;                                     // Set the Timeout parameters
    timeouts.ReadTotalTimeoutConstant=MAXDWORD;                         // No TimeOut
    timeouts.ReadTotalTimeoutMultiplier=0;
    timeouts.WriteTotalTimeoutConstant=MAXDWORD;
    timeouts.WriteTotalTimeoutMultiplier=0;
    if(!SetCommTimeouts(hSerial, &timeouts))                            // Write the parameters
        return -6;                                                      // Error while writting the parameters


#endif
#ifdef __linux__

#if UART_RX_POLL_MODE
	int iRx = setPollMode(chPort, iBauds);
	if (iRx < 0)
		return iRx;
#else
	g_tv.tv_sec = 0;
	g_tv.tv_usec = 2 * 1000;//2ms

	int iRx = setInterruptMode(chPort, iBauds);
	if (iRx < 0)
		return iRx;
#endif

#endif

    flushReceiver();
    mHandlesOpen_ = true;
    return 1;                                                           // Opening successfull
}

#ifdef __linux__

int HC_serial::setPollMode(const char *chPort, const unsigned int iBauds)
{
	struct termios options;                                             // Structure with the device's options


   // Open device
	m_fd = open(chPort, O_RDWR | O_NOCTTY | O_NDELAY);                    // Open port
	if (m_fd == -1)
	{
		LOG_ERROR("errno=%d\n", errno);
		return -2;
	}                                                                   // If the device is not open, return -1

	fcntl(m_fd, F_SETFL, FNDELAY);                                        // Open the device in nonblocking mode


	bool bOther = false;
	speed_t         Speed;
	switch (iBauds)                                                      // Set the speed (Bauds)
	{
		case 110:     Speed = B110; break;
		case 300:     Speed = B300; break;
		case 600:     Speed = B600; break;
		case 1200:     Speed = B1200; break;
		case 2400:     Speed = B2400; break;
		case 4800:     Speed = B4800; break;
		case 9600:     Speed = B9600; break;
		case 19200:    Speed = B19200; break;
		case 38400:    Speed = B38400; break;
		case 57600:    Speed = B57600; break;
		case 115200:   Speed = B115200; break;
		case 230400:   Speed = B230400; break;
		default: bOther = true; break;
	}
	if (bOther)
	{
		return setLinuxOtherBaud(m_fd, iBauds);

	}
	else
	{
		//// Set parameters
		tcgetattr(m_fd, &options);                                            // Get the current options of the port
		bzero(&options, sizeof(options));                                   // Clear all the options

		cfsetispeed(&options, Speed);
		cfsetospeed(&options, Speed);

		options.c_cflag |= (CLOCAL | CREAD | CS8);                        // Configure the device : 8 bits, no parity, no control
		options.c_iflag |= (IGNPAR | IGNBRK);
		options.c_cc[VTIME] = 0;                                              // Timer unused
		options.c_cc[VMIN] = 0;                                               // At least on character before satisfy reading
		tcsetattr(m_fd, TCSANOW, &options);                                   // Activate the settings
	}
	
	

	return 0;
}
int HC_serial::setInterruptMode(const char *chPort, const unsigned int iBauds)
{
	struct termios options;
	m_fd = open(chPort, O_RDWR | O_NOCTTY | O_NONBLOCK);// | O_NOCTTY | O_NDELAY
	if (m_fd == -1)
	{
		LOG_ERROR("errno=%d\n", errno);
		return -2;
	}
	g_fd = m_fd;

	g_saio.sa_handler = signal_handler_IO;
	sigemptyset(&g_saio.sa_mask);
	g_saio.sa_flags = 0;
	g_saio.sa_restorer = NULL;
	sigaction(SIGIO, &g_saio, NULL);

	fcntl(m_fd, F_SETFL, FNDELAY);
	fcntl(m_fd, F_SETOWN, getpid());
	fcntl(m_fd, F_SETFL, O_NDELAY | O_ASYNC); /**<<<<<<------This line made it work.**/


	bool bOther = false;
	speed_t         Speed;
	switch (iBauds)                                                      // Set the speed (Bauds)
	{
		case 110:     Speed = B110; break;
		case 300:     Speed = B300; break;
		case 600:     Speed = B600; break;
		case 1200:     Speed = B1200; break;
		case 2400:     Speed = B2400; break;
		case 4800:     Speed = B4800; break;
		case 9600:     Speed = B9600; break;
		case 19200:    Speed = B19200; break;
		case 38400:    Speed = B38400; break;
		case 57600:    Speed = B57600; break;
		case 115200:   Speed = B115200; break;
		case 230400:   Speed = B230400; break;
		default: bOther = true; break;
	}
	if (bOther)
	{
		int iRX = setLinuxOtherBaud(m_fd, iBauds);
		if (iRX < 0)
			return iRX;
	}
	else
	{
		tcgetattr(m_fd, &options);                                            // Get the current options of the port
		bzero(&options, sizeof(options));                                   // Clear all the options
		
		cfsetispeed(&options, Speed);
		cfsetospeed(&options, Speed);
		options.c_cflag &= ~PARENB;
		options.c_cflag &= ~CSTOPB;
		options.c_cflag &= ~CSIZE;
		options.c_cflag |= CS8;
		options.c_cflag |= (CLOCAL | CREAD);
		options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
		options.c_iflag &= ~(IXON | IXOFF | IXANY);
		options.c_oflag &= ~OPOST;
		tcsetattr(m_fd, TCSANOW, &options);
		//printf("serial configured....\n");

	}

	
	return 0;

}

int HC_serial::setLinuxOtherBaud(int fd,int iBauds)
{
	int status;
	struct termios options;
	struct serial_struct Serial;
	tcgetattr(fd, &options); /*Get current options*/
	tcflush(fd, TCIOFLUSH);/*Flush the buffer*/
	cfsetispeed(&options, B38400);/*Set input speed,38400 is necessary? who can tell me why?*/
	cfsetospeed(&options, 38400); /*Set output speed*/
	tcflush(fd, TCIOFLUSH); /*Flush the buffer*/
	status = tcsetattr(fd, TCSANOW, &options);
	/*Set the 38400 Options*/
	if (status != 0)
	{
		LOG_ERROR("tcsetattr fd1");
		return -4;
	}
	if ((ioctl(fd, TIOCGSERIAL, &Serial)) < 0)/*Get configurations vim IOCTL*/
	{
		LOG_ERROR("Fail to get Serial!\n");
		return -4;
	}
	Serial.flags = ASYNC_SPD_CUST;/*We will use custom buad,May be standard,may be not */
	Serial.custom_divisor = Serial.baud_base / iBauds;/*In Sep4020,baud_base=sysclk/16*/
	LOG_INFO("divisor is %x\n", Serial.custom_divisor);
	if ((ioctl(fd, TIOCSSERIAL, &Serial)) < 0)/*Set it*/
	{
		LOG_ERROR("Fail to set Serial\n");
		return -4;
	}
	ioctl(fd, TIOCGSERIAL, &Serial);/*Get it again,not necessary.*/
	LOG_INFO("\nBAUD: success set baud to %d,custom_divisor=%d,baud_base=%d\n", iBauds, Serial.custom_divisor, Serial.baud_base);
	
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	options.c_cflag |= (CLOCAL | CREAD);
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	options.c_iflag &= ~(IXON | IXOFF | IXANY);
	options.c_oflag &= ~OPOST;
	tcsetattr(m_fd, TCSANOW, &options);
	return 0;
}


#endif
/*!
\brief Close the connection with the current device
*/
void HC_serial::closeDevice()
{
    if (!mHandlesOpen_)
    {
        return;
    }
#if defined (_WIN32) || defined( _WIN64)
    if (hSerial != INVALID_HANDLE_VALUE)
    {
        CloseHandle(hSerial);
		hSerial = INVALID_HANDLE_VALUE;
    }
#endif
#ifdef __linux__
    close (m_fd);
    m_fd = 0;
#endif

    mHandlesOpen_ = false;
}

char HC_serial::writeChar(const char Byte)
{
#if defined (_WIN32) || defined( _WIN64)
    DWORD dwBytesWritten;                                               // Number of bytes written
    if(!WriteFile(hSerial,&Byte,1,&dwBytesWritten,NULL))                // Write the char
        return -1;                                                      // Error while writing
    return 1;                                                           // Write operation successfull
#endif
#ifdef __linux__
    if (write(m_fd,&Byte,1)!=1)                                           // Write the char
        return -1;                                                      // Error while writting
    return 1;                                                           // Write operation successfull
#endif
}


int HC_serial::writeData2(unsigned char * pData, int iLen)
{
#if defined (_WIN32) || defined( _WIN64)
	if (hSerial == INVALID_HANDLE_VALUE)
		return 0;

	//清空串口
	PurgeComm(hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);

	//写串口
	DWORD dwWrite = 0;
	bool bOK = WriteFile(hSerial, pData, iLen, &dwWrite, NULL);

	return dwWrite;

#endif
#ifdef __linux__
	if (m_fd == 0)
		return 0;

	if (write(m_fd, pData, iLen) != iLen)                                           // Write the char
		return 0;                                                      // Error while writting
	return iLen;

#endif
}

int HC_serial::readData( unsigned char *Buffer, unsigned int expectedBytes,int iTimeoutms)
{
    #if defined (_WIN32) || defined(_WIN64)
    if (hSerial == INVALID_HANDLE_VALUE)
    {
        return -5;
    }

    timeouts.ReadTotalTimeoutConstant = iTimeoutms;                       // Set the TimeOut
    if (!SetCommTimeouts(hSerial, &timeouts))                            // Write the parameters
        return -4;                                                      // Error while writting the parameters

    DWORD dwBytesRead = 0;
    if (!ReadFile(hSerial, Buffer, expectedBytes, &dwBytesRead, NULL))                 // Read the byte
    {
        CloseHandle(hSerial);
        hSerial = INVALID_HANDLE_VALUE;
        return -1;                                                      // Error while reading the byte
    }
    return dwBytesRead;
    #endif

 #ifdef __linux__
    

#if UART_RX_POLL_MODE
    int nfds;
    int nread = 0 ;
    
    fd_set readfds;
    struct timeval tv;

	if (m_fd == 0)
		return 0;

    tv.tv_sec = 0;
    tv.tv_usec = iTimeoutms*1000;//10ms
    
    FD_ZERO(&readfds);
    FD_SET(m_fd,&readfds);
    
    nfds = select(m_fd+1,&readfds,NULL,NULL,&tv);

    if(nfds == 0) 
    {
        return 0;
    }
    else
    {
        nread = read(m_fd , Buffer, expectedBytes);//即使不满desire_get_len，也会返回实际读取到的数据量
        return nread;
    }
#else
	std::lock_guard<std::mutex> lock(g_mtxBuff);

	int iSize = g_lstRx.size();
	//LOG_WARNING("Read data size=%d\n", iSize);
	if (iSize == 0)
	{
		return 0;
	}
	if (expectedBytes >= iSize)
	{
		for (int i = 0; i < iSize; i++)
		{
			Buffer[i] = g_lstRx.at(i);
		}

		std::vector<UCHAR> lstTmp;
		lstTmp.swap(g_lstRx);

		return iSize;
	}
	else
	{
		for (int i = 0; i < expectedBytes; i++)
		{
			Buffer[i] = g_lstRx.at(i);
		}
		HCHead::eraseBuff(g_lstRx, expectedBytes);

		return expectedBytes;
	}
	

#endif
    
 #endif
}




int HC_serial::readChar(unsigned char *pByte, unsigned int TimeOut_ms)
{
    return readChars(pByte, 1, TimeOut_ms);
}

int HC_serial::readChars(unsigned char *Buffer, unsigned int MaxNbBytes, unsigned int TimeOut_ms)
{
    int readBytes = 0;
    
    double timeElased = 0;
    int64_t startElased = GetTimeStamp();
    unsigned int alreadyReadBytes = 0;
    
    while (timeElased < TimeOut_ms && alreadyReadBytes != MaxNbBytes)
    {
        
        readBytes = readData(Buffer + alreadyReadBytes, MaxNbBytes - alreadyReadBytes);
        if (readBytes < 0)
        {
            printf("error:%d\n", readBytes);
            return readBytes;
        }

        alreadyReadBytes += readBytes;
        timeElased = (GetTimeStamp() - startElased) / 1.0e6;
        
    }

    if (alreadyReadBytes != 0 && alreadyReadBytes != MaxNbBytes)
    {
        return -3;
    }

    return alreadyReadBytes;
}

// _________________________
// ::: Special operation :::

/*!
\brief Empty receiver buffer (UNIX only)
*/

void HC_serial::flushReceiver()
{
#ifdef __linux__
    tcflush(m_fd, TCIOFLUSH);//tcflush(m_fd,TCIFLUSH);//tcflush(m_fd, TCIOFLUSH);
#endif
}

/*!
\brief  Return the number of bytes in the received buffer (UNIX only)
\return The number of bytes in the received buffer
*/
int HC_serial::peekReceiver()
{
    int Nbytes = 0;

#if defined (_WIN32) || defined(_WIN64)
    DWORD   errors = CE_IOE;
    COMSTAT commStat;

    if(!ClearCommError(hSerial, &errors, &commStat))
        Nbytes = 0;
    else
        Nbytes = commStat.cbInQue;
#endif
#ifdef __linux__
    ioctl(m_fd, FIONREAD, &Nbytes);
#endif
    return Nbytes;
}

// __________________
// ::: I/O Access :::

/*!
\brief      Set or unset the bit DTR
\param      Status=true  set DTR
Status=false unset DTR
*/
void HC_serial::DTR(bool Status)
{
#if defined (_WIN32) || defined(_WIN64)
    if(Status)
        EscapeCommFunction(hSerial, SETDTR);
    else
        EscapeCommFunction(hSerial, CLRDTR);
#endif
#ifdef __linux__

    int status_DTR=0;
    ioctl(m_fd, TIOCMGET, &status_DTR);
    if (Status)
        status_DTR |= TIOCM_DTR;
    else
        status_DTR &= ~TIOCM_DTR;
    ioctl(m_fd, TIOCMSET, &status_DTR);
#endif
}



/*!
\brief      Set or unset the bit RTS
\param      Status=true  set RTS
Status=false unset RTS
*/
void HC_serial::RTS(bool Status)
{
#if defined (_WIN32) || defined(_WIN64)
    if(Status)
        EscapeCommFunction(hSerial, SETRTS);
    else
        EscapeCommFunction(hSerial, CLRRTS);
#endif
#ifdef __linux__
    int status_RTS=0;
    ioctl(m_fd, TIOCMGET, &status_RTS);
    if (Status)
        status_RTS |= TIOCM_RTS;
    else
        status_RTS &= ~TIOCM_RTS;
    ioctl(m_fd, TIOCMSET, &status_RTS);
#endif
}




/*!
\brief      Get the CTS's status
\return     Return true if CTS is set otherwise false
*/
bool HC_serial::isCTS()
{
#if defined (_WIN32) || defined(_WIN64)
    DWORD dwModemStatus;

    if (!GetCommModemStatus(hSerial, &dwModemStatus))
        return false;
    if(MS_CTS_ON & dwModemStatus)
        return true;
    return false;
#endif
#ifdef __linux__
    int status=0;
    //Get the current status of the CTS bit
    ioctl(m_fd, TIOCMGET, &status);
    return status & TIOCM_CTS;
#endif
}



/*!
\brief      Get the CTS's status
\return     Return true if CTS is set otherwise false
*/
bool HC_serial::isDTR()
{
#if defined (_WIN32) || defined(_WIN64)
    DWORD dwModemStatus;

    if (!GetCommModemStatus(hSerial, &dwModemStatus))
        return false;
    if(MS_DSR_ON & dwModemStatus)
        return true;
    return false;
#endif
#ifdef __linux__
    int status=0;
    //Get the current status of the CTS bit
    ioctl(m_fd, TIOCMGET, &status);
    return status & TIOCM_DTR  ;
#endif
}

/*!
\brief      Get the CTS's status
\return     Return true if CTS is set otherwise false
*/
bool HC_serial::isRTS()
{
#if defined (_WIN32) || defined(_WIN64)
    DWORD dwModemStatus;

    if (!GetCommModemStatus(hSerial, &dwModemStatus))
        return false;
    if(MS_CTS_ON & dwModemStatus)
        return true;
    return false;
#endif
#ifdef __linux__
    int status=0;
    //Get the current status of the CTS bit
    ioctl(m_fd, TIOCMGET, &status);
    return status & TIOCM_RTS;
#endif
}

int64_t HC_serial::GetTimeStamp()
{
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    std::chrono::nanoseconds ns = now.time_since_epoch();

    return ns.count();
}
