/*!
\file    serial-O-matic.h
\brief   Header file of the class rOc_serial. This class is used for communication over a serial device.
\author  Philippe Lucidarme (University of Angers)
\version 1.2
\date    28 avril 2011
This Serial library is used to communicate through serial port.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE X CONSORTIUM BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

This is a licence-free software, it can be used by anyone who try to build a better world.
*/


#ifndef HC_SERIAL_H
#define HC_SERIAL_H



#define  UART_RX_POLL_MODE      1

// Used for TimeOut operations
//#include <sys/time.h>
// Include for windows
#if defined (_WIN32) || defined( _WIN64)
// Accessing to the serial port under Windows
#include <windows.h>
#include <stdint.h>
#else
#include <stdlib.h>
#include <sys/types.h>
#include <sys/shm.h>
#include <termios.h>
#include <string.h>
#include <iostream>
// File control definitions
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#endif


/*!  \class     rOc_serial
\brief     This class is used for communication over a serial device.
*/
class HC_serial
{
public:
	HC_serial();

    // Destructor
    ~HC_serial();

    int64_t GetTimeStamp();
    // Open a device
    char openDevice (const char *chPort, unsigned int iBauds);

    // Close the current device
    void closeDevice();

    char writeChar(const char Byte);
	int  writeData2(unsigned char * pData, int iLen);

    int readChar(unsigned char *pByte,const unsigned int TimeOut_ms=0);

    int readChars( unsigned char *Buffer, unsigned int MaxNbBytes, const unsigned int TimeOut_ms = 0);
    int readData( unsigned char *Buffer, unsigned int expectedBytes,int iTimeoutms=10);

    // Empty the received buffer
    void flushReceiver();

    // Return the number of bytes in the received buffer
    int peekReceiver();

    // Change CTR status
    void DTR(bool Status);

    // Change RTS status
    void RTS(bool Status);

    // Get CTS bit
    bool isCTS();

    // Get DTR bit
    bool isDTR();

    // Get CTS bit
    bool isRTS();
private:
    bool mHandlesOpen_;

#if defined (_WIN32) || defined( _WIN64)
    HANDLE hSerial;
    COMMTIMEOUTS timeouts;
#endif
#ifdef __linux__
    int m_fd;
	int setPollMode(const char *chPort, const unsigned int iBauds);
	int setInterruptMode(const char *chPort, const unsigned int iBauds);
	int setLinuxOtherBaud(int fd,int iBauds);
#endif

};



#endif // HC_SERIAL_H
