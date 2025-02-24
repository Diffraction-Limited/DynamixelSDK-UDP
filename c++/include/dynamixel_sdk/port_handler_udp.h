#include "port_handler.h"

#include <chrono>
#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <memory>

#ifdef _WIN32
#include "WinSock2.h"
#else
#include "sys/socket.h"
#include "arpa/inet.h"
#include "unistd.h"
#include <cstring>

#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define ADDR_ANY INADDR_ANY
#define closesocket close
#define SOCKET int
#define DWORD unsigned long
#endif
using sc_t = std::chrono::system_clock;

class PortHandlerUDP : public dynamixel::PortHandler 
{
public:

    PortHandlerUDP();

    /*
     * @description Constructor for PortHandlerUDP
     * @param ip null-terminated IP address of device to connect with
     * @param port port number device is listening on
     */
    PortHandlerUDP(const char* hostname);
    PortHandlerUDP(const char* device_ip, unsigned short device_port);
    virtual ~PortHandlerUDP();

    virtual bool openPort() override;
    virtual void closePort() override;
    virtual void clearPort() override;
    virtual void setPortName(const char *port_name) override;
    virtual char* getPortName() override;
    virtual bool setBaudRate(const int baudrate) override;
    virtual int getBaudRate() override;
    virtual int getBytesAvailable() override;
    virtual int readPort(uint8_t *packet, int length) override;
    virtual int writePort(uint8_t *packet, int length) override;
    virtual void setPacketTimeout(uint16_t packet_length) override;
    virtual void setPacketTimeout(double msec) override;
    virtual bool isPacketTimeout() override;

    void setDeviceIp(const char* ip);
    void setDevicePort(unsigned short port);

    const char* getDeviceIp() const;
    unsigned short getDevicePort() const;
    
    void setRxPortRange(unsigned short start, unsigned short end);
    unsigned short getRxPortStart() const;
    unsigned short getRxPortEnd() const;

protected:

    /// @brief  getAvailablePort
    /// @return the first available port in the range, or 0 if no ports are available
    ///
    /// This function will return the first available port in the range specified by setRxPortRange()
    unsigned short getAvailablePort() const;

    void setAbort(bool abort);
    bool getAbort() const;

    void setRunning(bool ready);
    bool isRunning() const;

    void setSocket(SOCKET socket);
    SOCKET getSocket() const;

    /// @brief appendPacket
    /// @param packet a pointer to a buffer that contains a received message.
    /// @param length the length in bytes of packet.
    ///
    /// This function makes a copy of packet and appends it to the receive queue. 
    /// This function is thread-safe and is blocking.
    void appendPacket(const uint8_t* packet, size_t length);

    /// @brief clearPackets
    /// 
    /// This function clears the receive buffer of all received messages.
    void clearPackets();
    
    /// @brief popPacket
    /// @param packet pointer to a buffer that will receive a copy of the topmost message on the Rx buffer
    /// @param length reference to a variable that will receive the length of the topmost message on the Rx buffer
    ///
    /// This function pops the topmost message from the receive buffer and copies it to the provided buffer.
    /// If the buffer is empty, this function will return without modifying the provided buffer.
    /// If the buffer is not empty, the topmost message will be copied to the provided buffer and removed from the Rx buffer.
    /// This function is thread-safe and will not block. 
    void popPacket(const uint8_t* packet, size_t& length); 

private:
    std::string m_portName = "";
    std::string m_ip = "";
    unsigned short m_port = 0, m_minPort = 10000, m_maxPort = 10010;
    SOCKET m_socket = INVALID_SOCKET;
    int m_baudRate = 9600;

    using TMessage = std::vector<uint8_t>;

    mutable std::mutex m_mutex;
    std::vector<TMessage> m_packetsIn = {};
    bool m_isRunning = false;
    bool m_abort = false;
    std::shared_ptr<std::thread> m_pWorker = nullptr;
    sc_t::time_point m_packetTimeout = sc_t::now();
};