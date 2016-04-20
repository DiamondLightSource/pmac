/*
 * PMAC Console
 *
 * Communicate with PMAC via Ethernet.
 *
 * Author: Klemen Zagar (2006-01-04)
 *
 * Compile with:
 *   gcc pmacConsole.c pmacEthernet.c -o pmacConsole
 */

#include <stdio.h>

#include "pmacEthernet.h"

int main(int argc, char *argv[])
{
	struct hostent *he;
	int sockfd;
	fd_set readfds;
	char buf[1400];
	char req[1400];
	char *trim, *tmp;
	char *host = argv[1];
	int read;
	int i;

	if(argc != 2) {
		printf("Usage: \n");
		printf("%s pmac-IP\n", argv[0]);
		return 2;
	}

	he = gethostbyname(host);
	if(he == NULL) {
		printf("Error resolving host %s\n", host);
		return 1;
	}

	sockfd = pmacSockOpen(he, &readfds);
	if(sockfd == -1) {
		printf("Error establishing communication with host %s\n", host);
		return 1;
	}
	pmacSockFlush(sockfd, &readfds);

	if(pmacSockGetResponse(sockfd, &readfds, "I6=1\n", buf) == -1) {
		printf("Error while communicating (I6=1) with PMAC!\n");
		return 1;
	}

	if(pmacSockGetResponse(sockfd, &readfds, "I3=2\0", buf) == -1) {
		printf("Error while communicating (I3=2) with PMAC!\n");
		return 1;
	}

	printf("Connected to PMAC at %s.\n", host);

	while(1) {
		fgets(req, sizeof(req), stdin);
		if(strcasecmp(req, "quit\n") == 0) {
			break;
		}

		trim = req;
		while(isspace(*trim)) {
			++trim;
		}
		tmp = trim;
		while(*tmp != 0) {
			++tmp;
		}
		--tmp;

		while(tmp >= trim) {
			if(isspace(*tmp)) {
				*tmp = 0;
			} else {
				break;
			}
			--tmp;
		}

		if(trim[0] == 0) {
			continue;
		}

		printf("STR: '%s'\n", trim);

		if((read = pmacSockGetResponse(sockfd, &readfds, trim, buf)) == -1) {
			printf("COMMUNICATION ERROR!\n");
		} else {
			for(i = 0; i < read; ++i) {
				if(buf[i] < 32) {
					printf(" $%02x ", (int)buf[i]);
				} else {
					fputc(buf[i], stdout);
				}
			}
			printf("\n");
		}
	}

	pmacSockClose(sockfd);
	return 0;
}
