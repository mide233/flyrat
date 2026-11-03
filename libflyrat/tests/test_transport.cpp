// SPDX-License-Identifier: MIT
//
// Transport smoke tests: TCP against a loopback echo server and a serial port
// against a Linux pseudo-terminal. No hardware required.
#include <arpa/inet.h>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <netinet/in.h>
#include <poll.h>
#include <pty.h>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

#include "flyrat/hardware/serial_transport.hpp"
#include "flyrat/hardware/tcp_transport.hpp"

namespace {

int g_failures = 0;

void check(bool condition, const std::string& what)
{
    if (!condition) {
        std::printf("FAIL: %s\n", what.c_str());
        ++g_failures;
    }
}

bool wait_readable(int fd, int timeout_ms)
{
    struct pollfd pfd {};
    pfd.fd = fd;
    pfd.events = POLLIN;
    return ::poll(&pfd, 1, timeout_ms) > 0;
}

void test_tcp_loopback()
{
    const int listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    check(listen_fd >= 0, "tcp: create listener");
    if (listen_fd < 0) {
        return;
    }

    struct sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    check(::bind(listen_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0,
        "tcp: bind loopback");
    check(::listen(listen_fd, 1) == 0, "tcp: listen");

    socklen_t addr_len = sizeof(addr);
    check(::getsockname(listen_fd, reinterpret_cast<sockaddr*>(&addr), &addr_len) == 0,
        "tcp: getsockname");
    const int port = ntohs(addr.sin_port);

    std::atomic<bool> server_ok { false };
    std::thread server([&]() {
        const int client = ::accept(listen_fd, nullptr, nullptr);
        if (client < 0) {
            return;
        }
        char buffer[16] = {};
        const ssize_t n = ::recv(client, buffer, sizeof(buffer), 0);
        if (n == 4 && std::memcmp(buffer, "ping", 4) == 0) {
            server_ok = ::send(client, "pong", 4, 0) == 4;
        }
        ::close(client);
    });

    auto transport = flyrat::hardware::make_tcp_transport("127.0.0.1", port, 1000);
    std::string error;
    check(transport->open(error), "tcp: open: " + error);
    check(transport->is_open(), "tcp: is_open");
    check(transport->native_handle() >= 0, "tcp: native handle");

    check(transport->write_all(reinterpret_cast<const std::uint8_t*>("ping"), 4, 1000, error),
        "tcp: write: " + error);

    std::uint8_t reply[8] = {};
    std::ptrdiff_t got = 0;
    if (wait_readable(transport->native_handle(), 1000)) {
        got = transport->read_some(reply, sizeof(reply), error);
    }
    check(got == 4 && std::memcmp(reply, "pong", 4) == 0, "tcp: loopback echo");

    transport->close();
    check(!transport->is_open(), "tcp: close");

    server.join();
    ::close(listen_fd);
    check(server_ok.load(), "tcp: server saw the request");
}

void test_serial_pty()
{
    int master = -1;
    int slave = -1;
    check(::openpty(&master, &slave, nullptr, nullptr, nullptr) == 0, "pty: openpty");
    if (master < 0 || slave < 0) {
        return;
    }

    char name[256] = {};
    check(::ttyname_r(slave, name, sizeof(name)) == 0, "pty: ttyname_r");
    ::close(slave);

    auto transport = flyrat::hardware::make_serial_transport(name, 115200);
    std::string error;
    check(transport->open(error), "serial: open: " + error);

    check(::write(master, "abc", 3) == 3, "pty: write to master");
    std::uint8_t buffer[8] = {};
    std::ptrdiff_t got = 0;
    if (wait_readable(transport->native_handle(), 1000)) {
        got = transport->read_some(buffer, sizeof(buffer), error);
    }
    check(got == 3 && std::memcmp(buffer, "abc", 3) == 0, "serial: read from pty");

    check(transport->write_all(reinterpret_cast<const std::uint8_t*>("xyz"), 3, 1000, error),
        "serial: write: " + error);
    std::uint8_t reply[8] = {};
    got = 0;
    if (wait_readable(master, 1000)) {
        got = ::read(master, reply, sizeof(reply));
    }
    check(got == 3 && std::memcmp(reply, "xyz", 3) == 0, "serial: write to pty");

    transport->close();
    check(!transport->is_open(), "serial: close");
    ::close(master);
}

} // namespace

int main()
{
    test_tcp_loopback();
    test_serial_pty();

    if (g_failures != 0) {
        std::printf("%d transport check(s) failed\n", g_failures);
        return 1;
    }
    std::printf("transport tests passed\n");
    return 0;
}
