#include <signal.h>

int spi_pru_loader(void);

int gShouldStop = 0;
void catch_function(int signo){
	gShouldStop = 1;
}

int main(void){
	signal(SIGINT, catch_function);
	return spi_pru_loader();
}

