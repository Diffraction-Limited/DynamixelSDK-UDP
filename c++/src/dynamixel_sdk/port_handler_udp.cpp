#include "port_handler_udp.h"

#ifdef _WIN32
#include "WinSock2.h"
#include "Ws2tcpip.h"
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

#define LATENCY_TIMER 100 // 100 msec

using lockguard_t = std::lock_guard<std::mutex>;

PortHandlerUDP::PortHandlerUDP()
    : dynamixel::PortHandler()
{
}

PortHandlerUDP::PortHandlerUDP(const char *hostname)
    : dynamixel::PortHandler()
    , m_ip("")
    , m_port(0)
{
    std::string hostname_str(hostname);
    std::string prefix = "udp://";
    if (hostname_str.compare(0, prefix.size(), prefix) == 0) {
        size_t ip_start = prefix.size();
        size_t port_start = hostname_str.find(':', ip_start);
        if (port_start != std::string::npos) {
            m_ip = hostname_str.substr(ip_start, port_start - ip_start);
            m_port = static_cast<unsigned short>(std::stoi(hostname_str.substr(port_start + 1)));
        } else {
            m_ip = hostname_str.substr(ip_start);
            m_port = 6464;
        }
    } else {
        throw std::invalid_argument("Invalid hostname format: missing udp:// prefix");
    }
}

PortHandlerUDP::PortHandlerUDP(const char *ip, unsigned short port)
    : dynamixel::PortHandler()
    , m_ip(ip)
    , m_port(port)
{

}

PortHandlerUDP::~PortHandlerUDP() 
{ 
    closePort();
}

// Thread-safe accessors
void PortHandlerUDP::setAbort(bool abort)
{
    lockguard_t lock(m_mutex);
    m_abort = abort;
}

bool PortHandlerUDP::getAbort() const
{
    lockguard_t lock(m_mutex);
    return m_abort;
}

void PortHandlerUDP::setRunning(bool running)
{
    lockguard_t lock(m_mutex);
    m_isRunning = running;
}

bool PortHandlerUDP::isRunning() const
{
    lockguard_t lock(m_mutex);
    return m_isRunning;
}

void PortHandlerUDP::setSocket(SOCKET socket)
{
    lockguard_t lock(m_mutex);
    m_socket = socket;
}

SOCKET PortHandlerUDP::getSocket() const
{
    lockguard_t lock(m_mutex);
    return m_socket;
}

void PortHandlerUDP::appendPacket(const uint8_t* packet, size_t length)
{
    TMessage data(length);
    memcpy(data.data(), packet, length);
    
    lockguard_t lock(m_mutex);
    m_packetsIn.push_back(data);
}

void PortHandlerUDP::clearPackets()
{
    lockguard_t lock(m_mutex);
    m_packetsIn.clear();
}

void PortHandlerUDP::popPacket(const uint8_t* packet, size_t& length)
{
    // Default to an error condition
    length = -1;

    // copy front packet into buffer & clear it from the Rx queue
    lockguard_t lock(m_mutex);
    if (m_packetsIn.empty()) return;
    auto data = m_packetsIn.front();
    length = data.size();
    memcpy((void*)packet, data.data(), length);
    m_packetsIn.erase(m_packetsIn.begin());
}

unsigned short PortHandlerUDP::getAvailablePort() const
{
    unsigned short port = m_minPort, maxPort = m_maxPort;
    while (port <= maxPort)
    {
        struct sockaddr_in server;
        server.sin_family = AF_INET;
        server.sin_port = htons(port);
        inet_pton(AF_INET, m_ip.c_str(), &server.sin_addr);

        SOCKET sock = INVALID_SOCKET;
        if (bind(sock, (struct sockaddr*)&server, sizeof(server)) == 0)
        {
            closesocket(sock);
            break;
        }

        port++;
    }
    if (port > maxPort)
    {
        port = 0;
    }
    return port;
}

bool compareIP(sockaddr_in& a, sockaddr_in& b)
{
#ifdef _WIN32
    return a.sin_addr.S_un.S_addr == b.sin_addr.S_un.S_addr;
#else
    return a.sin_addr.s_addr == b.sin_addr.s_addr;
#endif
}

bool PortHandlerUDP::openPort()
{
    // Only proceed if the worker thread is not already running
    if (isRunning()) return true;

    // Start the worker thread
    m_pWorker = std::make_shared<std::thread>([&](){
        // Open and configure the UDP socket
        auto sock = getSocket();
        if (sock != INVALID_SOCKET)
        {
            // If sock is not INVALID_SOCKET, then there's another thread running with a socket open
            // we need to close that and block until it's done.
            closePort();
        }

        sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock == INVALID_SOCKET)
        {
            setSocket(INVALID_SOCKET);
            setRunning(false);
            return;
        }

        setSocket(sock);

        struct in_addr src_ip;
        auto src_port = getDevicePort();

        if (src_port == 0)
        {
            // Failing to find an open port in range is a critical error
            setSocket(INVALID_SOCKET);
            setRunning(false);
            return;
        }

        // Convert the stored version to a sockaddr_in
        inet_pton(AF_INET, getDeviceIp(), &src_ip);
        struct sockaddr_in src_addr;
        src_addr.sin_family = AF_INET;
        src_addr.sin_port = htons(src_port);
        src_addr.sin_addr = src_ip;

        // Set socket options
        DWORD socketRxTimeOut_ms = 1000; // 1 second timeout
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&socketRxTimeOut_ms, sizeof(socketRxTimeOut_ms));

        // Set up the socket bindings
        auto listenPort = getAvailablePort();
        struct sockaddr_in server;
        server.sin_family = AF_INET;
        server.sin_port = htons(listenPort);
        server.sin_addr.s_addr = ADDR_ANY;

        // Bind the socket to our listening port
        if (bind(sock, (struct sockaddr*)&server, sizeof(server)) < 0)
        {
            // Failing to bind is a critical error
            setSocket(INVALID_SOCKET);
            setRunning(false);
            return;
        }

        // Report that we're running
        setRunning(true);
        while (true)
        {
            // Sleep for a bit to prevent this thread from hogging the CPU
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(1ms);

            // Abort if requested
            if (getAbort())
            {
                setAbort(false);
                break;
            }

            // Check for incoming packets
            uint8_t packet[1024] = {0};
            sockaddr_in client; int client_len = sizeof(client);
            int bytesIn = recvfrom(sock, (char*)packet, 1024, 0, (struct sockaddr*)&client, (socklen_t*)&client_len);
            if (bytesIn > 0)
            {
                // Ignore packets from invalid sources & ports
                if (!compareIP(src_addr, client)) continue;
                if (src_addr.sin_port != client.sin_port) continue;
                
                // append valid packets to the Rx buffer
                appendPacket(packet, bytesIn);
            }
        }

        // Abort has completed, close the socket and report that we're no longer running
        closesocket(sock);
        setRunning(false);
    });

    return true;
}

void PortHandlerUDP::closePort()
{
    setAbort(true);
    if (m_pWorker && m_pWorker->joinable())
    {   
        m_pWorker->join();
    }
    m_pWorker = nullptr;
    return;
}

void PortHandlerUDP::clearPort()
{    
    clearPackets();
}

void PortHandlerUDP::setPortName(const char *port_name)
{
    m_portName = std::string(port_name, strlen(port_name));
}

char* PortHandlerUDP::getPortName()
{
    return const_cast<char*>(m_portName.c_str());
}

bool PortHandlerUDP::setBaudRate(const int baudrate)
{
    m_baudRate = baudrate;

    // TODO : Communicate this to the device

    return true;
}

int PortHandlerUDP::getBaudRate()
{
    return m_baudRate;
}

int PortHandlerUDP::getBytesAvailable()
{
    lockguard_t lock(m_mutex);
    if (m_packetsIn.empty()) return 0;
    return (int)m_packetsIn.front().size();
}

int PortHandlerUDP::readPort(uint8_t *packet, int length)
{
    // Clear the incoming buffer
    memset(packet, 0, length);

    // Peek to see if we have an incoming packet
    size_t packetInLng = getBytesAvailable();

    // exit if incoming buffer is too small
    if ((int)packetInLng > length) return -1;
    
    // copy incoming mesasge to buffer and return its length as required by dynamixel sdk
    popPacket(packet, packetInLng);
    return (int)packetInLng;
}

int PortHandlerUDP::writePort(uint8_t *packet, int length)
{
    auto sock = getSocket();
    if (sock == INVALID_SOCKET)
    {
        return -1;
    }

    // set up destination address & port (device IP & port)
    auto ip = getDeviceIp();
    auto port = getDevicePort();

    struct sockaddr_in server;
    server.sin_family = AF_INET;
    server.sin_port = htons(port);
    inet_pton(AF_INET, ip, &server.sin_addr);

    // NOTE: sendto returns -1 on error, otherwise bytes written, which matches up with dynamixel sdk expectations
    return sendto(sock, (const char*)packet, length, 0, (struct sockaddr*)&server, sizeof(server));
}

void PortHandlerUDP::setPacketTimeout(uint16_t packet_length)
{
    if (m_baudRate <= 9600) m_baudRate = 9600;
    auto tx_time_per_byte = (1000.0 / (double)m_baudRate) * 10.0;
    auto timeout_ms = 10. * (tx_time_per_byte * (double)packet_length) + (LATENCY_TIMER * 2.0) + 2.0;
    setPacketTimeout(timeout_ms);    
}

void PortHandlerUDP::setPacketTimeout(double msec)
{
    auto timeout_ms = static_cast<int>(msec); // cast from double to int
    m_packetTimeout = sc_t::now() + std::chrono::milliseconds(timeout_ms);
}

bool PortHandlerUDP::isPacketTimeout()
{
    return sc_t::now() > m_packetTimeout;
}
