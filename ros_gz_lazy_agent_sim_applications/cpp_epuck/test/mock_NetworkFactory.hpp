#include <cpp_epuck/NetworkFactory.hpp>
#include <gmock/gmock.h>

// Mock implementations for testing
class MockIoContext : public IIoContext
{
public:
    MOCK_METHOD(void, run, (), (override));
    MOCK_METHOD(void, stop, (), (override));
    MOCK_METHOD(boost::asio::io_context &, getNativeContext, (), (override));

    // This is needed for tests that must interact with the real io_context
    boost::asio::io_context realContext;

    boost::asio::io_context &getNativeContextImpl() { return realContext; }
};

class MockTcpSocket : public ITcpSocket
{
public:
    MOCK_METHOD(void, connect, (const boost::asio::ip::tcp::endpoint &endpoint), (override));
    MOCK_METHOD(void, asyncConnect,
                (const boost::asio::ip::tcp::endpoint &endpoint,
                 std::function<void(const boost::system::error_code &)>),
                (override));
    MOCK_METHOD(size_t, send, (const boost::asio::const_buffer &buffer), (override));
    MOCK_METHOD(void, asyncSend,
                (const boost::asio::const_buffer &buffer,
                 std::function<void(const boost::system::error_code &, std::size_t)>),
                (override));
    MOCK_METHOD(size_t, receive, (const boost::asio::mutable_buffer &buffer), (override));
    MOCK_METHOD(void, asyncReceive,
                (const boost::asio::mutable_buffer &buffer,
                 std::function<void(const boost::system::error_code &, std::size_t)>),
                (override));
    MOCK_METHOD(void, close, (), (override));
    MOCK_METHOD(boost::asio::ip::tcp::socket &, getNativeSocket, (), (override));

    // Helper methods for tests
    void simulateReceive(const std::vector<char> &data, const boost::system::error_code &ec = {})
    {
        if (receiveHandler)
        {
            std::memcpy(receiveBuffer.data(), data.data(), std::min(data.size(), receiveBuffer.size()));
            receiveHandler(ec, std::min(data.size(), receiveBuffer.size()));
        }
    }

    void captureReceiveBuffer(const boost::asio::mutable_buffer &buffer,
                              std::function<void(const boost::system::error_code &, std::size_t)> handler)
    {
        receiveBuffer  = boost::asio::buffer(buffer);
        receiveHandler = handler;
    }

private:
    boost::asio::mutable_buffer receiveBuffer;
    std::function<void(const boost::system::error_code &, std::size_t)> receiveHandler;
};

class MockUdpSocket : public IUdpSocket
{
public:
    MOCK_METHOD(void, open, (), (override));
    MOCK_METHOD(void, bind, (const boost::asio::ip::udp::endpoint &endpoint), (override));
    MOCK_METHOD(size_t, sendTo,
                (const boost::asio::const_buffer &buffer, const boost::asio::ip::udp::endpoint &destination),
                (override));
    MOCK_METHOD(void, asyncSendTo,
                (const boost::asio::const_buffer &buffer, const boost::asio::ip::udp::endpoint &destination,
                 std::function<void(const boost::system::error_code &, std::size_t)>),
                (override));
    MOCK_METHOD(size_t, receiveFrom,
                (const boost::asio::mutable_buffer &buffer, boost::asio::ip::udp::endpoint &sender), (override));
    MOCK_METHOD(void, asyncReceiveFrom,
                (const boost::asio::mutable_buffer &buffer, boost::asio::ip::udp::endpoint &sender,
                 std::function<void(const boost::system::error_code &, std::size_t)>),
                (override));
    MOCK_METHOD(void, close, (), (override));
    MOCK_METHOD(boost::asio::ip::udp::socket &, getNativeSocket, (), (override));

    // Helper methods for tests
    void simulateReceive(const std::vector<char> &data, const boost::system::error_code &ec = {})
    {
        if (receiveHandler)
        {
            std::memcpy(receiveBuffer.data(), data.data(), std::min(data.size(), receiveBuffer.size()));
            receiveHandler(ec, std::min(data.size(), receiveBuffer.size()));
        }
    }

    void captureReceiveBuffer(const boost::asio::mutable_buffer &buffer,
                              std::function<void(const boost::system::error_code &, std::size_t)> handler)
    {
        receiveBuffer  = boost::asio::buffer(buffer);
        receiveHandler = handler;
    }

private:
    boost::asio::mutable_buffer receiveBuffer;
    std::function<void(const boost::system::error_code &, std::size_t)> receiveHandler;
};

class MockResolver : public IResolver
{
public:
    MOCK_METHOD(boost::asio::ip::tcp::resolver::results_type, resolveTcp,
                (const std::string &host, const std::string &service), (override));
    MOCK_METHOD(
        void, asyncResolveTcp,
        (const std::string &host, const std::string &service,
         std::function<void(const boost::system::error_code &, const boost::asio::ip::tcp::resolver::results_type &)>
             handler),
        (override));
    MOCK_METHOD(boost::asio::ip::udp::resolver::results_type, resolveUdp,
                (const std::string &host, const std::string &service), (override));
    MOCK_METHOD(
        void, asyncResolveUdp,
        (const std::string &host, const std::string &service,
         std::function<void(const boost::system::error_code &, const boost::asio::ip::udp::resolver::results_type &)>
             handler),
        (override));
};

class MockTimer : public ITimer
{
public:
    MOCK_METHOD(void, expires_after, (const boost::asio::steady_timer::duration &expiry_time), (override));
    MOCK_METHOD(void, expires_at, (const boost::asio::steady_timer::time_point &expiry_time), (override));
    MOCK_METHOD(void, wait, (), (override));
    MOCK_METHOD(void, asyncWait, (std::function<void(const boost::system::error_code &)> handler), (override));
    MOCK_METHOD(void, cancel, (), (override));
};

// Mock factory for testing
class MockNetworkFactory : public INetworkFactory
{
public:
    // These methods will return mock objects that can be configured in tests
    std::unique_ptr<IIoContext> createIoContext() override
    {
        auto mock = std::make_unique<MockIoContext>();
        // Set up default behavior
        ON_CALL(*mock, getNativeContext())
            .WillByDefault(testing::Invoke(mock.get(), &MockIoContext::getNativeContextImpl));
        return mock;
    }

    std::unique_ptr<ITcpSocket> createTcpSocket(IIoContext &ioContext) override
    {
        return std::make_unique<MockTcpSocket>();
    }

    std::unique_ptr<IUdpSocket> createUdpSocket(IIoContext &ioContext) override
    {
        // Similar implementation for UDP socket mocks
        return std::make_unique<MockUdpSocket>();
    }

    std::unique_ptr<IResolver> createResolver(IIoContext &ioContext) override
    {
        return std::make_unique<MockResolver>();
    }

    std::unique_ptr<ITimer> createTimer(IIoContext &ioContext) override { return std::make_unique<MockTimer>(); }
};
