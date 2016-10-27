
#ifndef sshDriver_H
#define sshDriver_H

#include "libssh2_config.h"
#include <libssh2.h>

#ifdef HAVE_WINSOCK2_H
# include <winsock2.h>
#endif
#ifdef HAVE_SYS_SOCKET_H
# include <sys/socket.h>
#endif
#ifdef HAVE_NETINET_IN_H
# include <netinet/in.h>
#endif
# ifdef HAVE_UNISTD_H
#include <unistd.h>
#endif
#ifdef HAVE_ARPA_INET_H
# include <arpa/inet.h>
#endif
#ifdef HAVE_SYS_TIME_H
# include <sys/time.h>
#endif

#include <sys/types.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <ctype.h>

typedef enum e_SSHDriverStatus
{
  SSHDriverSuccess,
  SSHDriverError
} SSHDriverStatus;

/**
 * The SSHDriver class provides a wrapper around the libssh2 library.
 * It takes out some of the complexity of creating SSH connections and
 * provides a simple read/write/flush interface.  Setting up a connection
 * can be configured with a host name/IP, username and optional password.
 *
 * @author Alan Greer (ajg@observatorysciences.co.uk)
 */
class SSHDriver {

  public:
    SSHDriver(const char *host);
    SSHDriverStatus setUsername(const char *username);
    SSHDriverStatus setPassword(const char *password);
    SSHDriverStatus connectSSH();
    SSHDriverStatus flush();
    SSHDriverStatus write(const char *buffer, size_t bufferSize, size_t *bytesWritten, int timeout);
    SSHDriverStatus read(char *buffer, size_t bufferSize, size_t *bytesRead, int readTerm, int timeout);
    SSHDriverStatus syncInteractive(const char *snd_str,  const char *exp_str);
    SSHDriverStatus disconnectSSH();
    virtual ~SSHDriver();

  private:
    int sock_;
    int auth_pw_;
    int connected_;
    struct sockaddr_in sin_;
    LIBSSH2_SESSION *session_;
    LIBSSH2_CHANNEL *channel_;
    char host_[256];
    char username_[256];
    char password_[256];
    off_t got_;

    SSHDriverStatus setBlocking(int blocking);

};


#endif


