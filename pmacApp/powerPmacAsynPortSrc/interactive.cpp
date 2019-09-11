#include "sshDriver.h"

int main(int argc, char *argv[])
{
  SSHDriver *ptr = new SSHDriver("172.23.247.1");
  ptr->setUsername("root");
  ptr->setPassword("deltatau");
  ptr->connectSSH();

  char buff[512];
  size_t bytes = 0;
  // Write vers<CR>
  strcpy(buff, "gpascii\n");
  ptr->write(buff, strlen(buff), &bytes, 1000);
  ptr->read(buff, 512, &bytes, '\n', 2000);
  printf("We got the following:\n");
  printf("%s", buff);
  printf("\n");
  sleep(1);

  int c;
  int counter = 0;
  char cmd[1024];

  counter = 0;
  do {
    counter = 0;
    do {
      c = getchar();
      cmd[counter] = c;
      counter++;
    } while(c != '\n');
    // Append the terminator
    cmd[counter] = '\0';
    if (strcmp(cmd, "quit")){
      ptr->write(cmd, strlen(cmd), &bytes, 1000);
      ptr->read(cmd, 1024, &bytes, 0x06, 1000);
      //ptr->writeRead(cmd, strlen(cmd), cmd, &bytes, 0x0A, 0x06, 1000);
      cmd[strlen(cmd)-2] = '\0';
      printf("%s\n", cmd);
    }
    // Quit if we get a quit
  } while(strcmp(cmd, "quit"));
  return 0;

}
