/********************************************
 *  sshDriver.cpp
 *
 *  SSH wrapper class for the libssh2 library
 *  This class provides standard read/write
 *  and flush methods for an ssh connection.
 *
 *  Alan Greer
 *  21 Jan 2013
 *
 ********************************************/

#include "sshDriver.h"
#include <sys/time.h>

#include <osiUnistd.h>
#include <osiSock.h>
#include <poll.h>
#include <stdlib.h>

#define SSH_PORT 22
#define SSH_NOT_CONNECTED 0
#define SSH_CONNECTED 1
#define SSH_ERROR_TIMEOUT_MS 100

#define PPMAC_MAX_CHAR_LENGTH 7168
#define PPMAC_DOUBLE_MAX_LENGTH 2*PPMAC_MAX_CHAR_LENGTH

/*
 * Uncomment the DEBUG define and recompile for lots of
 * driver debug messages.
 * Uncomment LOGCOM if you want to see the communication
 */
//#define DEBUG 1
//#define LOGCOM 1

#ifdef WIN32
  #define Close(s)  closesocket(s)
#else
  #define Close(s)  close(s)
#endif


#if defined(DEBUG) || defined(LOGCOM)
void PrintEscapedNL(const char *buff, size_t bytes)
{
  for (unsigned int j = 0; j < bytes; j++){
    char ch =  buff[j];
    if (isprint(ch)) {
      printf("%c", ch);
    } else if (ch == '\\') {
      printf("\\\\");
    } else if (ch == '\t') {
      printf("\\t");
    } else if (ch == '\n') {
      printf("\\n");
    } else if (ch == '\r') {
      printf("\\r");
    } else {
      printf("\\%03o", ch);
    }
  }
  printf("\n");
}
#endif


#ifdef DEBUG
#define debugPrint printf
#define debugStrPrintEscapedNL(a,b) PrintEscapedNL((a),(b))
#else
void debugPrint(...){}
void debugStrPrintEscapedNL(const char *buff, size_t bytes)
{
  (void)buff;
  (void)bytes;
}
#endif

#ifdef LOGCOM
#define LogComPrint printf
#define LogComStrPrintEscapedNL(a,b) PrintEscapedNL((a),(b))
#else
void LogComPrint(...){}
void LogComStrPrintEscapedNL(const char *buff, size_t bytes)
{
  (void)buff;
  (void)bytes;
}
#endif

/**
 * Constructor for the SSH driver.  Accepts a host name or IP
 * address.  The class will attempt to resolve the name to an
 * IP address before connecting.  Initializes internal variables.
 *
 * @param host - Host name/IP to attempt a connection with.
 */
SSHDriver::SSHDriver(const char *host)
{
  static const char *functionName = "SSHDriver::SSHDriver";
  debugPrint("%s : Method called\n", functionName);

  // Initialize internal SSH parameters
  auth_pw_ = 0;
  got_ = 0;
  connected_ = SSH_NOT_CONNECTED;
  // Initialise username and password
  username_ = NULL;
  password_ = NULL;

  // Allocate memory required for the write method buffers
  write_input_ = (char *)malloc(PPMAC_MAX_CHAR_LENGTH);
  read_buffer_ = (char *)malloc(PPMAC_DOUBLE_MAX_LENGTH);
  write_expected_echo_ = (char *)malloc(PPMAC_DOUBLE_MAX_LENGTH);

  // Allocate the memory required to store the host address
  host_ = (char *)malloc(strlen(host));
  // Store the host address
  strcpy(host_, host);

  error_checking_ = false;
  potential_errors_ = 0;
  caught_errors_ = 0;
  caught_delays_ = 0;
}

/**
 * Setup the username for the connection.  Obviously the
 * username must exist on the device running the SSH
 * server.
 *
 * @param username - Username for the SSH connection.
 * @return - Success or failure.
 */
SSHDriverStatus SSHDriver::setUsername(const char *username)
{
  static const char *functionName = "SSHDriver::setUsername";
  debugPrint("%s : Method called\n", functionName);

  // Allocate the memory for the username
  if (username_){
    free(username_);
  }
  username_ = (char *)malloc(strlen(username));

  // Store the username
  strcpy(username_, username);

  return SSHDriverSuccess;
}

/**
 * Setup the password for the username on this connection.
 * A password does not need to be entered.  If it is not then
 * key based authorization will be attempted.
 *
 * @param password - Password for the SSH connection.
 * @return - Success or failure.
 */
SSHDriverStatus SSHDriver::setPassword(const char *password)
{
  static const char *functionName = "SSHDriver::setPassword";
  debugPrint("%s : Method called\n", functionName);

  // Allocate the memory for the password
  if (password_){
    free(password_);
  }
  password_ = (char *)malloc(strlen(password));

  // Store the password
  strcpy(password_, password);

  // Set the password authentication to on
  auth_pw_ = 1;

  return SSHDriverSuccess;
}

/**
 * Attempt to create a connection and authorize the username
 * with the password (or by keys).  Once the connection has
 * been established a dumb terminal is created and an attempt
 * to read the initial welcome lines is made.
 *
 * @return - Success or failure.
 */
SSHDriverStatus SSHDriver::connectSSH()
{
  unsigned long hostaddr;
  int rc;
  int i;
  static const char *functionName = "SSHDriver::connect";
  debugPrint("%s : Method called\n", functionName);

#ifdef WIN32
  WSADATA wsadata;

  WSAStartup(MAKEWORD(2,0), &wsadata);
#endif

  in_addr inhost;
  if (hostToIPAddr(host_, &inhost) < 0){
    debugPrint("%s : libssh2 unknown host %s\n", functionName, host_);
    return SSHDriverError;
  }
  hostaddr = inhost.s_addr;
  debugPrint("%s : String host address (%s)\n", functionName, host_);
  debugPrint("%s : libssh2 host address (%ld)\n", functionName, hostaddr);

  rc = libssh2_init(0);
  if (rc != 0) {
    debugPrint("%s : libssh2 initialization failed, error code (%d)\n", functionName, rc);
    return SSHDriverError;
  }

  // Create the socket neccessary for the connection
  sock_ = socket(AF_INET, SOCK_STREAM, 0);

  sin_.sin_family = AF_INET;
  sin_.sin_port = htons(SSH_PORT);
  sin_.sin_addr.s_addr = hostaddr;
  if (connect(sock_, (struct sockaddr*)(&sin_), sizeof(struct sockaddr_in)) != 0){
    debugPrint("%s : socket failed to connect!\n", functionName);
    Close(sock_);
    return SSHDriverError;
  }

  // Create a session instance
  session_ = libssh2_session_init();
  if(!session_){
    debugPrint("%s : libssh2 failed to create a session instance\n", functionName);
    Close(sock_);
    return SSHDriverError;
  }

  // Start up the session. This will trade welcome banners, exchange keys,
  // and setup crypto, compression, and MAC layers
  rc = libssh2_session_handshake(session_, sock_);
  if(rc){
    debugPrint("%s : libssh2 failure establishing SSH session: %d\n", functionName, rc);
    Close(sock_);
    return SSHDriverError;
  }

  // Here we now have a connection that will need to be closed
  connected_ = SSH_CONNECTED;

  // At this point the connection hasn't yet authenticated.  The first thing to do
  // is check the hostkey's fingerprint against the known hosts.
  const char *fingerprint = libssh2_hostkey_hash(session_, LIBSSH2_HOSTKEY_HASH_SHA1);
  debugPrint("%s : SSH fingerprint: ", functionName);
  for(i = 0; i < 20; i++) {
    debugPrint("%02X ", (unsigned char)fingerprint[i]);
  }
  debugPrint("\n");

  if (auth_pw_ == 1){
    // Authenticate via password
    if (libssh2_userauth_password(session_, username_, password_)) {
      debugPrint("%s : SSH authentication by password failed.\n", functionName);
      disconnectSSH();
      return SSHDriverError;
    } else {
      debugPrint("%s : SSH authentication by password worked.\n", functionName);
    }
  } else {
    /* Or by public key */
    char rsapubbuff[256];
    char rsabuff[256];
    sprintf(rsapubbuff, "/home/%s/.ssh/id_rsa.pub", username_);
    sprintf(rsabuff, "/home/%s/.ssh/id_rsa", username_);
    if (libssh2_userauth_publickey_fromfile(session_, username_, rsapubbuff, rsabuff, password_)){
      debugPrint("%s : SSH authentication by public key failed\n", functionName);
      disconnectSSH();
      return SSHDriverError;
    }
  }

  libssh2_trace(session_, LIBSSH2_TRACE_CONN);

  // Open the channel for read/write
  channel_ = libssh2_channel_open_session(session_);
  debugPrint("%s : SSH channel opened\n", functionName);

  // Request a terminal with 'dumb' terminal emulation
  // See /etc/termcap for more options
  if (libssh2_channel_request_pty(channel_, "dumb")){
    debugPrint("%s : Failed requesting dumb pty\n", functionName);
    disconnectSSH();
    return SSHDriverError;
  }

  // Open a SHELL on that pty
  if (libssh2_channel_shell(channel_)) {
    debugPrint("%s : Unable to request shell on allocated pty\n", functionName);
    disconnectSSH();
    return SSHDriverError;
  }

  setBlocking(0);

  long tnow = 0;
  timeval stime;
  timeval ctime;
  gettimeofday(&stime, NULL);

  // Poll the underlying socket for bytes once connection established
  // Do not read or write using libssh2 until the socket has received
  // bytes from the server.  These first bytes will contain the welcome
  // message and then it is safe to proceed with reading and writing
  // through the established connection.
	const char numfds = 1;
	struct pollfd pfds[numfds];
	memset(pfds, 0, sizeof(struct pollfd) * numfds);
  bool first_read = false;
  debugPrint("Poll underlying socket for first bytes...\n");
  while (!first_read){
		pfds[0].fd = sock_;
		pfds[0].events = POLLIN;
		pfds[0].revents = 0;
		rc = poll(pfds, numfds, -1);  
		if (-1 == rc) {
			perror("poll");
			break;
		}
		if (pfds[0].revents & POLLIN) {
      first_read = true;
    } else {
      debugPrint("Polled underlying socket, no bytes ready for reading.\n");
    }
  }

  gettimeofday(&ctime, NULL);
  tnow = ((ctime.tv_sec - stime.tv_sec) * 1000) + ((ctime.tv_usec - stime.tv_usec) / 1000);
  debugPrint("Time taken for first read to arrive: %ld ms\n", tnow);

  // Here we should wait for the initial welcome line
  char buffer[1024];
  size_t bytes = 0;
  debugPrint("Pre-read the buffer for any characters.\n");
  read(buffer, 512, &bytes, 0x06, 500, false);
  buffer[bytes] = 0;
  debugPrint("Buffer read: %s\n", buffer);

  debugPrint("Cycle through the prompt, calling PS1=!?\%#\n");
  const char *ps1_last_txt = "!?%#";
  for (unsigned int i=0; i <strlen(ps1_last_txt); i++)
  {
    // Set the prompt and read it back
    sprintf(buffer, "PS1=%c\n", ps1_last_txt[i]);

    debugPrint("Setting prompt command to: %s\n", buffer);
    write(buffer, strlen(buffer), &bytes, 1000);
    read(buffer, 512, &bytes, ps1_last_txt[i], 3000, false);
    buffer[bytes] = '\0';
    debugPrint("Read back: %s\n", buffer);
  }
  debugPrint("Completed cycling through the command prompt.\n");
  /* Read the final '\n' */
  read(buffer, 512, &bytes, '\n', 100, false);
  debugPrint("%s : Connection ready...\n", functionName);

  return SSHDriverSuccess;
}

/**
 * Set the connection to a blocking or non-blocking connection.
 *
 * @param blocking - 0 for non-blocking or 1 for blocking.
 * @return - Success or failure.
 */
SSHDriverStatus SSHDriver::setBlocking(int blocking)
{
  static const char *functionName = "SSHDriver::setBlocking";
  debugPrint("%s : Method called\n", functionName);

  // Make the channel blocking or non-blocking
  libssh2_channel_set_blocking(channel_, blocking);
  debugPrint("%s : Set blocking value to %d\n", functionName, blocking);
  return SSHDriverSuccess;
}

/**
 * Flush the connection as best as possible.
 *
 * @return - Success or failure.
 */
SSHDriverStatus SSHDriver::flush()
{
  static const char *functionName = "SSHDriver::flush";
  debugPrint("%s : Method called\n", functionName);

  if (connected_ == SSH_NOT_CONNECTED){
    debugPrint("%s : Not connected\n", functionName);
    return SSHDriverError;
  }

  // Call the underlying libssh2 flush for all channel streams
  int rc = libssh2_channel_flush_ex(channel_, LIBSSH2_CHANNEL_FLUSH_ALL);
  if (rc < 0){
    debugPrint("Flush: libssh2_channel_flush_ex failed with error code %d\n", rc);
    return SSHDriverError;
  }
  // Read out any remaining bytes from t he channel
  rc = libssh2_channel_read(channel_, read_buffer_, PPMAC_DOUBLE_MAX_LENGTH);
  if (rc > 0){
    debugPrint("Flushed %d bytes\n", rc);
  }

  if (rc < 0){
    debugPrint("Flush: libssh2_channel_read failed with error code %d\n", rc);
    return SSHDriverError;
  }
  return SSHDriverSuccess;
}

SSHDriverStatus SSHDriver::setErrorChecking(bool error_check)
{
  error_checking_ = error_check;
  return SSHDriverSuccess;
}

/**
 * Write data to the connected channel.  A timeout should be
 * specified in milliseconds.
 *
 * @param buffer - The string buffer to be written.
 * @param bufferSize - The number of bytes to write.
 * @param bytesWritten - The number of bytes that were written.
 * @param timeout - A timeout in ms for the write.
 * @return - Success or failure.
 */
SSHDriverStatus SSHDriver::write(const char *buffer, size_t bufferSize, size_t *bytesWritten, int timeout)
{
  static const char *functionName = "SSHDriver::write";
  debugPrint("%s : Method called\n", functionName);
  *bytesWritten = 0;

  if (connected_ == SSH_NOT_CONNECTED){
    debugPrint("%s : Not connected\n", functionName);
    return SSHDriverError;
  }

  if (bufferSize > PPMAC_MAX_CHAR_LENGTH){
    debugPrint("%s : write request exceeds the maximum buffer size\n", functionName);
    return SSHDriverError;
  }

  timeval stime;
  timeval ctime;
  gettimeofday(&stime, NULL);
  long mtimeout = (stime.tv_usec / 1000) + timeout;
  long tnow = 0;

  strncpy(write_input_, buffer, bufferSize);
  write_input_[bufferSize] = 0;
  flush();
  LogComPrint("LogCom sshDriver Writing %02lu bytes => ", (unsigned long)bufferSize);
  LogComStrPrintEscapedNL(buffer, bufferSize);

  int rc = libssh2_channel_write(channel_, write_input_, bufferSize);
  if (rc > 0){
    debugPrint("%s : %d bytes written\n", functionName, rc);
    *bytesWritten = rc;
  } else {
    debugPrint("%s : No bytes were written, libssh2 error (%d)\n", functionName, rc);
    *bytesWritten = 0;
    return SSHDriverError;
  }

  // Now we need to read back the same numer of bytes, to remove the written string from the buffer
  // Each of these buffers must be twice that of the input buffer to allow for the PowerPMAC responding
  // with a \r\n for every \n that is written by the request
  int bytesToRead = *bytesWritten;
  int bytes = 0;
  rc = 0;
  int crCount = 0;

  // Count the number of \n characters sent
  // Build the expected ECHO string
  memset(write_expected_echo_, 0, PPMAC_DOUBLE_MAX_LENGTH);
  int expected_index = 0;
  for (int index = 0; index < (int)*bytesWritten; index++){
    if (buffer[index] == '\n'){
      // CR, need to read back 1 extra character
      crCount++;
      write_expected_echo_[expected_index] = '\r';
      expected_index++;
    }
    write_expected_echo_[expected_index] = buffer[index];
    expected_index++;
  }
  bytesToRead += crCount;
  int matched = 0;
  while ((matched == 0) && (tnow < mtimeout)){
    rc = libssh2_channel_read(channel_, &read_buffer_[bytes], bytesToRead);
    if (rc > 0){
      bytes+=rc;
      bytesToRead-=rc;

      // Loop over end of strings in reverse, to check for a match
      if (bytes >= expected_index){
        matched = 1;
        for (int mid = 1; mid <= expected_index; mid++){
          if (read_buffer_[bytes-mid] != write_expected_echo_[expected_index-mid]){
            matched = 0;
          }
        }
      }
    }
    if (bytesToRead > 0 || matched == 0){
      usleep(50);
      gettimeofday(&ctime, NULL);
      tnow = ((ctime.tv_sec - stime.tv_sec) * 1000) + (ctime.tv_usec / 1000);
    }
  }

  if (error_checking_){
    if (read_buffer_[0] == '\r' && write_input_[0] != '\r' && expected_index > 2){
      caught_errors_++;
      debugPrint("Caught communication error\n");
      debugPrint("Matched status: %d\n", matched);
      debugPrint("Input string: ");
      for (int index=0; index <= expected_index; index++){
        debugPrint("[%d] ", write_input_[index]);
      }
      debugPrint("\n");
      debugPrint("Expected response: ");
      for (int index=0; index <= expected_index; index++){
        debugPrint("[%d] ", write_expected_echo_[index]);
      }
      debugPrint("\n");
      debugPrint("Actual response: ");
      for (int index=0; index <= expected_index; index++){
        debugPrint("[%d] ", read_buffer_[index]);
      }
      debugPrint("\n");
    }
  }

  read_buffer_[bytes] = '\0';

  LogComPrint("LogCom sshDriver Echoed  %02d bytes => ", bytes);
  LogComStrPrintEscapedNL(read_buffer_, bytes);


  gettimeofday(&ctime, NULL);
  tnow = ((ctime.tv_sec - stime.tv_sec) * 1000) + (ctime.tv_usec / 1000);
  debugPrint("%s : Time taken for write => %ld ms\n", functionName, (tnow-(mtimeout-timeout)));
  if (tnow >= mtimeout){
    return SSHDriverError;
  }

  return SSHDriverSuccess;
}

/**
 * Read data from the connected channel.  A timeout should be
 * specified in milliseconds.  The read method will continue to
 * read data from the channel until either the specified
 * terminator is read or the timeout is reached.
 *
 * @param buffer - A string buffer to hold the read data.
 * @param bufferSize - The maximum number of bytes to read.
 * @param bytesRead - The number of bytes that have been read.
 * @param readTerm - A terminator to use as a check for EOM (End Of Message).
 * @param timeout - A timeout in ms for the read.
 * @param crlf - Boolean. If true then match against the terminator followed by CRLF.
 * @return - Success or failure.
 */
SSHDriverStatus SSHDriver::read(char *buffer, size_t bufferSize, size_t *bytesRead, int readTerm, int timeout, bool crlf)
{
  static const char *functionName = "SSHDriver::read";
  char ch = readTerm;
  debugPrint("%s : Method called\n", functionName);
  debugPrint("%s : Read terminator %d ", functionName, readTerm);
  debugStrPrintEscapedNL(&ch, sizeof(ch));

  if (connected_ == SSH_NOT_CONNECTED){
    debugPrint("%s : Not connected\n", functionName);
    return SSHDriverError;
  }

  timeval stime;
  timeval ctime;
  gettimeofday(&stime, NULL);
  long mtimeout = (stime.tv_usec / 1000) + timeout;
  long tnow = stime.tv_usec / 1000;
  int rc = 0;
  int matched = 0;
  int matchedindex = 0;
  int lastCount = 0;
  *bytesRead = 0;
//#ifdef DEBUG
  memset(buffer, 0, bufferSize);
//#endif
  while ((matched == 0) && (tnow < mtimeout)){
    /* TODO: Make sure that this loop does not run over bufferSize when
     a match is not found and matched is never set to 1 */
    rc = libssh2_channel_read(channel_, &buffer[*bytesRead], (bufferSize-*bytesRead));
    if (rc > 0){
      *bytesRead+=rc;
    }
    // Catch instances where the previous version would have failed
    if (error_checking_){
      if (buffer[(*bytesRead) - 1] == readTerm){
        debugPrint("Captured potential failure in the previous software version\n");
        potential_errors_++;
      }
    }

    for (int index = lastCount; index < (int)*bytesRead; index++){
      if (crlf){
        // Match against output terminator
        if (buffer[index-2] == readTerm && buffer[index-1] == '\r' && buffer[index-0] == '\n'){
          matched = 1;
          matchedindex = index;
        }
      } else {
        if (buffer[index] == readTerm){
          matched = 1;
          matchedindex = index;
        }
      }
    }
    lastCount = *bytesRead;
    if (matched == 0){
      usleep(50);
      gettimeofday(&ctime, NULL);
      tnow = ((ctime.tv_sec - stime.tv_sec) * 1000) + (ctime.tv_usec / 1000);
    }
  }

  if (error_checking_){
    if ((mtimeout - tnow) < SSH_ERROR_TIMEOUT_MS){
      debugPrint("Delay in read response: %ld ms\n", (timeout - (mtimeout - tnow)));
      debugPrint("Input buffer: %s\n", buffer);
      caught_delays_++;
    }
  }

  //buffer[matchedindex] = '\0';
  debugPrint("%s : %d Bytes => ", functionName, lastCount);
  debugStrPrintEscapedNL(buffer, lastCount);

  if (matched == 1){
    lastCount = matchedindex+1;
  }
  *bytesRead = lastCount;
  debugPrint("%s : Matched %d lastCount=%d\n", functionName, matched, lastCount);
  LogComPrint("LogCom sshDriver Reading %02d bytes => ", lastCount);
  LogComStrPrintEscapedNL(buffer, lastCount);

  gettimeofday(&ctime, NULL);
  tnow = ((ctime.tv_sec - stime.tv_sec) * 1000) + (ctime.tv_usec / 1000);
  debugPrint("%s : Time taken for read => %ld ms\n", functionName, (tnow-(mtimeout-timeout)));

  if ((timeout > 0) && (tnow >= mtimeout)){
    return SSHDriverError;
  }

  return SSHDriverSuccess;
}


/**
 * Sync the connection.
 *
 * @return - Success or failure.
 */
SSHDriverStatus SSHDriver::syncInteractive(const char *snd_str,  const char *exp_str)
{
  static const char *functionName = "SSHDriver::syncInteractive";
  size_t exp_str_len = strlen(exp_str);
  char buff[512];
  size_t bytes = 0;
  SSHDriverStatus status = SSHDriverError;

  debugPrint("%s : Method called exp_str => ", functionName);
  debugStrPrintEscapedNL(exp_str, exp_str_len);

  debugPrint("%s : snd_str => ", functionName);
  debugStrPrintEscapedNL(snd_str, strlen(snd_str));

  for (unsigned int cnt = 0; cnt < 10; cnt++) {
    write(snd_str, strlen(snd_str), &bytes, 1000);
    buff[0] = 0;
    read(buff, sizeof(buff), &bytes, exp_str[exp_str_len-1], 1000);

    if (bytes == strlen(exp_str) && !memcmp(exp_str, buff, bytes)) {
      status = SSHDriverSuccess;
    }
    debugPrint("%s : status=%d bytes=%lu rec_str => ", functionName, (int)status, (unsigned long)bytes);
    debugStrPrintEscapedNL(buff, bytes);
    if (status == SSHDriverSuccess) {
      return status;
    }
  }

  return SSHDriverError;
}


/**
 * Close the connection.
 *
 * @return - Success or failure.
 */
SSHDriverStatus SSHDriver::disconnectSSH()
{
  static const char *functionName = "SSHDriver::disconnect";
  debugPrint("%s : Method called\n", functionName);

  if (connected_ == SSH_CONNECTED){
    connected_ = SSH_NOT_CONNECTED;
    libssh2_session_disconnect(session_, "Normal Shutdown");
    libssh2_session_free(session_);

    Close(sock_);
    debugPrint("%s : Completed disconnect\n", functionName);

    libssh2_exit();

  } else {
    debugPrint("%s : Connection was never established\n", functionName);
  }
  return SSHDriverSuccess;
}

/**
 * Destructor, cleanup.
 */
SSHDriver::~SSHDriver()
{
  static const char *functionName = "SSHDriver::~SSHDriver";
  debugPrint("%s : Method called\n", functionName);

  free(write_input_);
  free(read_buffer_);
  free(write_expected_echo_);

  free(host_);
  if (username_){
    free(username_);
  }
  if (password_){
    free(password_);
  }
}

SSHDriverStatus SSHDriver::report(FILE *fp)
{
  fprintf(fp, "    Potential error cases: %d\n", potential_errors_);
  fprintf(fp, "      (These might have been flushed or caused a real error)\n");
  fprintf(fp, "    Caught error cases that have been handled: %d\n", caught_errors_);
  fprintf(fp, "    Write/Reads that have take more than 100ms: %d\n", caught_delays_);
  return SSHDriverSuccess;
}
